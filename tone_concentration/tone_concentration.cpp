#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================================
// 定数定義
// ============================================================================

// ピン定義
namespace Pins {
  constexpr byte RX = 4;
  constexpr byte TX = 5;
  constexpr byte SERIAL_595 = 6;
  constexpr byte LATCH_595 = 7;
  constexpr byte CLK_595 = 8;
  constexpr byte START_BUTTON = 3;
}

// I2Cアドレス
namespace I2CAddr {
  constexpr byte LCD = 0x50;
  constexpr byte MCP23017 = 0x21;
}

// MCP23017レジスタ
namespace MCP23017Reg {
  constexpr byte IODIRA = 0x00;
  constexpr byte IODIRB = 0x01;
  constexpr byte GPIOA = 0x12;
  constexpr byte GPIOB = 0x13;
  constexpr byte GPPUA = 0x0C;
  constexpr byte GPPUB = 0x0D;
}

// マトリックス設定
namespace Matrix {
  constexpr byte ROWS = 4;
  constexpr byte COLS = 4;
  constexpr byte TOTAL_CELLS = ROWS * COLS;
}

// タイミング設定
namespace Timing {
  constexpr unsigned long SCAN_INTERVAL_MS = 100;
  constexpr unsigned long MATCH_CHECK_DELAY_MS = 1500;
  constexpr unsigned long LED_RESET_DELAY_MS = 2500;
  constexpr unsigned int LED_SCAN_INTERVAL_US = 2000; // 2ms, 500Hz
}

// 音声ファイル設定
namespace Audio {
  constexpr byte FOLDER_SYSTEM = 1;
  constexpr byte FOLDER_CARDS = 2;
  constexpr byte FILE_START = 1;
  constexpr byte FILE_MATCH = 2;
  constexpr byte FILE_NO_MATCH = 3;
  constexpr byte DEFAULT_VOLUME = 20;
}

// シリアル通信設定
namespace Serial {
  constexpr unsigned long BAUD_RATE_DEBUG = 19200;
  constexpr unsigned long BAUD_RATE_MP3 = 9600;
}

// I2C設定
namespace I2CConfig {
  constexpr unsigned long CLOCK_SPEED = 50000UL;
}

// ============================================================================
// グローバルオブジェクト
// ============================================================================

LiquidCrystal_I2C lcd(I2CAddr::LCD, 16, 2);
DFRobotDFPlayerMini mp3;
SoftwareSerial mp3Serial(Pins::RX, Pins::TX);

// ============================================================================
// ゲーム状態管理
// ============================================================================

enum class GameState : byte {
  WAITING_FOR_INPUT,
  CHECKING_MATCH
};

struct GameData {
  byte ledStates[Matrix::ROWS];
  byte cardNumbers[Matrix::ROWS][Matrix::COLS];
  byte selectedCards[2];
  byte selectedPositions[2];
  byte selectionCount;
  GameState state;
  unsigned long stateStartTime;
};

GameData game;

// ============================================================================
// マトリックスLED制御（タイマー割り込みベース）
// ============================================================================

namespace MatrixLED {
  const byte COL_DATA[Matrix::COLS] = {0x08, 0x04, 0x02, 0x01};
  const byte ROW_DATA[Matrix::ROWS] = {0x80, 0x40, 0x20, 0x10};
  
  volatile byte currentScanRow = 0;

  void write(byte bitOrder, byte data) {
    digitalWrite(Pins::LATCH_595, LOW);
    shiftOut(Pins::SERIAL_595, Pins::CLK_595, bitOrder, data);
    digitalWrite(Pins::LATCH_595, HIGH);
  }

  // タイマー割り込みから呼ばれるスキャン関数
  void scanNextRow() {
    write(LSBFIRST, game.ledStates[currentScanRow]);
    currentScanRow = (currentScanRow + 1) % Matrix::ROWS;
  }

  void turnOn(byte row, byte col) {
    if (row < Matrix::ROWS && col < Matrix::COLS) {
      game.ledStates[row] |= COL_DATA[col];
    }
  }

  void turnOff(byte row, byte col) {
    if (row < Matrix::ROWS && col < Matrix::COLS) {
      game.ledStates[row] &= ~COL_DATA[col];
    }
  }

  bool isOn(byte row, byte col) {
    if (row < Matrix::ROWS && col < Matrix::COLS) {
      return !(game.ledStates[row] & COL_DATA[col]);
    }
    return false;
  }
}

// ============================================================================
// I2C通信
// ============================================================================

namespace I2C {
  void writeRegister(byte deviceAddr, byte reg, byte data) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
  }

  byte readRegister(byte deviceAddr, byte reg) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.endTransmission();

    constexpr byte NUM_BYTES = 1;
    Wire.requestFrom(deviceAddr, NUM_BYTES);

    if (Wire.available()) {
      return Wire.read();
    } else {
      Serial.print(F("I2C Read Error - Reg: 0x"));
      Serial.println(reg, HEX);
      return 0xFF;
    }
  }

  void scanDevices() {
    Serial.println(F("I2C Device Scan:"));
    for (byte addr = 1; addr < 127; ++addr) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print(F("  Found: 0x"));
        Serial.println(addr, HEX);
      }
    }
  }
}

// ============================================================================
// MCP23017制御
// ============================================================================

namespace MCP23017 {
  void initialize() {
    // 全ピンを入力に設定
    I2C::writeRegister(I2CAddr::MCP23017, MCP23017Reg::IODIRA, 0xFF);
    I2C::writeRegister(I2CAddr::MCP23017, MCP23017Reg::IODIRB, 0xFF);
    
    // 内部プルアップを有効化
    I2C::writeRegister(I2CAddr::MCP23017, MCP23017Reg::GPPUA, 0xFF);
    I2C::writeRegister(I2CAddr::MCP23017, MCP23017Reg::GPPUB, 0xFF);
  }

  word readSwitchMatrix() {
    byte upperByte = I2C::readRegister(I2CAddr::MCP23017, MCP23017Reg::GPIOA);
    byte lowerByte = I2C::readRegister(I2CAddr::MCP23017, MCP23017Reg::GPIOB);
    return (lowerByte << 8) | upperByte;
  }
}

// ============================================================================
// カード管理
// ============================================================================

namespace Cards {
  struct Position {
    byte row;
    byte col;
    
    Position() : row(0), col(0) {}
    Position(byte r, byte c) : row(r), col(c) {}
  };

  Position getPositionFromSwitchData(word switchData) {
    if (switchData == 0xFFFF) {
      return Position(0xFF, 0xFF); // 無効な位置
    }

    unsigned int bit = 1;
    byte bitCount = 0;

    while (switchData & bit) {
      ++bitCount;
      bit <<= 1;
    }

    byte row = bitCount / Matrix::COLS;
    byte col = bitCount % Matrix::COLS;

    return Position(row, col);
  }

  void shuffle() {
    // ペアの番号を生成（1,1,2,2,3,3,...,8,8）
    byte numbers[Matrix::TOTAL_CELLS];
    byte cardValue = 1;
    
    for (byte i = 0; i < Matrix::TOTAL_CELLS; ++i) {
      numbers[i] = cardValue;
      if (i % 2 == 1) ++cardValue;
    }

    // Fisher-Yatesアルゴリズムでシャッフル
    randomSeed(analogRead(A0));
    for (int i = Matrix::TOTAL_CELLS - 1; i > 0; --i) {
      int j = random(0, i + 1);
      byte temp = numbers[i];
      numbers[i] = numbers[j];
      numbers[j] = temp;
    }

    // 2次元配列に変換
    for (byte row = 0; row < Matrix::ROWS; ++row) {
      for (byte col = 0; col < Matrix::COLS; ++col) {
        game.cardNumbers[row][col] = numbers[row * Matrix::COLS + col];
      }
    }
  }
}

// ============================================================================
// 音声制御
// ============================================================================

namespace AudioPlayer {
  bool initialize() {
    if (!mp3.begin(mp3Serial)) {
      Serial.println(F("MP3 Player initialization failed!"));
      return false;
    }
    mp3.volume(Audio::DEFAULT_VOLUME);
    return true;
  }

  void playSystemSound(byte fileNumber) {
    mp3.playFolder(Audio::FOLDER_SYSTEM, fileNumber);
  }

  void playCardSound(byte cardNumber) {
    mp3.playFolder(Audio::FOLDER_CARDS, cardNumber);
  }

  void playMatchResult(bool isMatch) {
    playSystemSound(isMatch ? Audio::FILE_MATCH : Audio::FILE_NO_MATCH);
  }
}

// ============================================================================
// LCD制御
// ============================================================================

namespace Display {
  void initialize() {
    lcd.init();
    lcd.clear();
    lcd.backlight();
  }

  void showMessage(const char* message, byte col = 0, byte row = 0) {
    lcd.setCursor(col, row);
    lcd.print(message);
  }

  void showWelcome() {
    showMessage("HELLO!", 5, 0);
  }
}

// ============================================================================
// タイマー割り込み設定
// ============================================================================

namespace Timer {
  // Timer1割り込みサービスルーチン
  ISR(TIMER1_COMPA_vect) {
    MatrixLED::scanNextRow();
  }

  void initialize() {
    // Timer1を使用してマトリックススキャン用の割り込みを設定
    noInterrupts(); // 割り込み無効化
    
    // Timer1レジスタをクリア
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    
    // CTC（Clear Timer on Compare Match）モード設定
    // OCR1Aの値と一致したらタイマーをリセット
    TCCR1B |= (1 << WGM12);
    
    // プリスケーラを8に設定
    // 16MHz / 8 = 2MHz = 0.5μs
    TCCR1B |= (1 << CS11);
    
    // 比較一致値を設定
    // 2ms間隔 = 2000μs / 0.5μs = 4000
    OCR1A = (F_CPU / 8UL) * Timing::LED_SCAN_INTERVAL_US / 1000000UL - 1;
    
    // Timer1比較一致割り込みを有効化
    TIMSK1 |= (1 << OCIE1A);
    
    interrupts(); // 割り込み有効化
    
    Serial.print(F("Timer initialized: OCR1A = "));
    Serial.println(OCR1A);
  }

  void start() {
    TIMSK1 |= (1 << OCIE1A);
  }

  void stop() {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}

namespace Game {
  void initialize() {
    game.state = GameState::WAITING_FOR_INPUT;
    game.selectionCount = 0;
    game.stateStartTime = millis();
    
    // LED状態を初期化
    for (byte i = 0; i < Matrix::ROWS; ++i) {
      game.ledStates[i] = MatrixLED::ROW_DATA[i];
    }
    
    Cards::shuffle();
  }

  void handleCardSelection(const Cards::Position& pos) {
    // 既に選択済みまたは点灯済みのカードは無視
    if (MatrixLED::isOn(pos.row, pos.col)) {
      return;
    }

    // カードを選択
    MatrixLED::turnOn(pos.row, pos.col);
    game.selectedCards[game.selectionCount] = game.cardNumbers[pos.row][pos.col];
    game.selectedPositions[game.selectionCount] = (pos.row << 4) | pos.col;
    
    // 音声再生
    AudioPlayer::playCardSound(game.selectedCards[game.selectionCount]);
    
    ++game.selectionCount;

    // 2枚選択されたら判定フェーズへ
    if (game.selectionCount >= 2) {
      game.state = GameState::CHECKING_MATCH;
      game.stateStartTime = millis();
    }
  }

  void checkMatch() {
    unsigned long elapsed = millis() - game.stateStartTime;
    
    // マッチ結果の音声再生
    if (elapsed >= Timing::MATCH_CHECK_DELAY_MS && game.selectionCount >= 2) {
      bool isMatch = (game.selectedCards[0] == game.selectedCards[1]);
      AudioPlayer::playMatchResult(isMatch);
      game.selectionCount = 0; // カウンターリセット
    }
    
    // 不一致の場合、LEDを消灯
    if (elapsed >= Timing::LED_RESET_DELAY_MS) {
      if (game.selectedCards[0] != game.selectedCards[1]) {
        for (byte i = 0; i < 2; ++i) {
          byte row = (game.selectedPositions[i] & 0xF0) >> 4;
          byte col = game.selectedPositions[i] & 0x0F;
          MatrixLED::turnOff(row, col);
        }
      }
      
      // 入力待ち状態に戻る
      game.state = GameState::WAITING_FOR_INPUT;
      game.stateStartTime = millis();
    }
  }

  void processInput() {
    unsigned long elapsed = millis() - game.stateStartTime;
    
    if (elapsed >= Timing::SCAN_INTERVAL_MS) {
      word switchData = MCP23017::readSwitchMatrix();
      
      if (switchData != 0xFFFF) {
        Cards::Position pos = Cards::getPositionFromSwitchData(switchData);
        
        if (pos.row < Matrix::ROWS && pos.col < Matrix::COLS) {
          handleCardSelection(pos);
        }
      }
      
      game.stateStartTime = millis();
    }
  }

// ============================================================================
// ゲームロジック
// ============================================================================

namespace Game {
  void initialize() {
    game.state = GameState::WAITING_FOR_INPUT;
    game.selectionCount = 0;
    game.stateStartTime = millis();
    
    // LED状態を初期化
    for (byte i = 0; i < Matrix::ROWS; ++i) {
      game.ledStates[i] = MatrixLED::ROW_DATA[i];
    }
    
    Cards::shuffle();
  }

  void handleCardSelection(const Cards::Position& pos) {
    // 既に選択済みまたは点灯済みのカードは無視
    if (MatrixLED::isOn(pos.row, pos.col)) {
      return;
    }

    // カードを選択
    MatrixLED::turnOn(pos.row, pos.col);
    game.selectedCards[game.selectionCount] = game.cardNumbers[pos.row][pos.col];
    game.selectedPositions[game.selectionCount] = (pos.row << 4) | pos.col;
    
    // 音声再生
    AudioPlayer::playCardSound(game.selectedCards[game.selectionCount]);
    
    ++game.selectionCount;

    // 2枚選択されたら判定フェーズへ
    if (game.selectionCount >= 2) {
      game.state = GameState::CHECKING_MATCH;
      game.stateStartTime = millis();
    }
  }

  void checkMatch() {
    unsigned long elapsed = millis() - game.stateStartTime;
    
    // マッチ結果の音声再生
    if (elapsed >= Timing::MATCH_CHECK_DELAY_MS && game.selectionCount >= 2) {
      bool isMatch = (game.selectedCards[0] == game.selectedCards[1]);
      AudioPlayer::playMatchResult(isMatch);
      game.selectionCount = 0; // カウンターリセット
    }
    
    // 不一致の場合、LEDを消灯
    if (elapsed >= Timing::LED_RESET_DELAY_MS) {
      if (game.selectedCards[0] != game.selectedCards[1]) {
        for (byte i = 0; i < 2; ++i) {
          byte row = (game.selectedPositions[i] & 0xF0) >> 4;
          byte col = game.selectedPositions[i] & 0x0F;
          MatrixLED::turnOff(row, col);
        }
      }
      
      // 入力待ち状態に戻る
      game.state = GameState::WAITING_FOR_INPUT;
      game.stateStartTime = millis();
    }
  }

  void processInput() {
    unsigned long elapsed = millis() - game.stateStartTime;
    
    if (elapsed >= Timing::SCAN_INTERVAL_MS) {
      word switchData = MCP23017::readSwitchMatrix();
      
      if (switchData != 0xFFFF) {
        Cards::Position pos = Cards::getPositionFromSwitchData(switchData);
        
        if (pos.row < Matrix::ROWS && pos.col < Matrix::COLS) {
          handleCardSelection(pos);
        }
      }
      
      game.stateStartTime = millis();
    }
  }

  void update() {
    switch (game.state) {
      case GameState::WAITING_FOR_INPUT:
        processInput();
        break;
        
      case GameState::CHECKING_MATCH:
        checkMatch();
        break;
    }
  }
}

// ============================================================================
// 初期化・メインループ
// ============================================================================

void waitForStartButton() {
  pinMode(Pins::START_BUTTON, INPUT);
  while (digitalRead(Pins::START_BUTTON));
  while (!digitalRead(Pins::START_BUTTON));
}

void setup() {
  // シリアル通信初期化
  Serial.begin(Serial::BAUD_RATE_DEBUG);
  mp3Serial.begin(Serial::BAUD_RATE_MP3);
  
  // I2C初期化
  Wire.begin();
  Wire.setClock(I2CConfig::CLOCK_SPEED);
  
  // 各モジュール初期化
  Display::initialize();
  Display::showWelcome();
  
  // ピン設定
  pinMode(Pins::SERIAL_595, OUTPUT);
  pinMode(Pins::LATCH_595, OUTPUT);
  pinMode(Pins::CLK_595, OUTPUT);
  
  // デバイス初期化
  if (!AudioPlayer::initialize()) {
    Serial.println(F("Setup failed!"));
    while (true);
  }
  
  MCP23017::initialize();
  
  // デバイススキャン（デバッグ用）
  I2C::scanDevices();
  
  // ゲーム初期化
  Game::initialize();
  
  // タイマー初期化（マトリックススキャン用）
  Timer::initialize();
  
  // 開始音再生
  AudioPlayer::playSystemSound(Audio::FILE_START);
  
  // スタートボタン待機
  waitForStartButton();
  
  Serial.println(F("Game Start!"));
}

void loop() {
  Game::update();
}

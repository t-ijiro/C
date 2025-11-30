/*********************************************************************************************/
//
//  FILE        : othello.c
//  DATE        : 2025/11/25 Tue.
//  DESCRIPTION : Main Program
//  CPU TYPE    : RX Family
//
//  Author T.Ijiro
//
//  ビルド
//  ・以下の割り込み関数をintprg.c内でコメントアウトする
//    Excep_CMT0_CMI0, Excep_CMT1_CMI1, Excep_CMT2_CMI2, Excep_ICU_IRQ0, Excep_ICU_IRQ1
//  ・stacksct.hのsuを0xFF0に変更する
//
//  入力機能
//  ・ロータリーエンコーダー : カーソル移動
//  ・sw5                  : 2〜3秒長押しでリセット
//  ・sw6                  : サウンドオンオフ
//  ・sw7                  : 決定
//  ・sw8                  : 移動方向オプション選択 ONで縦移動モード OFFで横移動モード
/************************************************************************************************/
// #include "typedefine.h"
#ifdef __cplusplus
// #include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

#include <string.h>
#include <stdlib.h>
#include <machine.h>
#include "iodefine.h"
#include "vect.h"
#include "lcd_lib4.h"
#include "onkai.h"

/************************************ マクロ *************************************************/
// 時間、周期
#define MONITOR_CHATTERING_PERIOD_MS 300  // チャタリング監視周期. IRQ用.
#define CURSOR_BLINK_PERIOD_MS       150  // カーソルの点滅周期
#define AI_MOVE_PERIOD_MS            300  // AIの移動周期
#define LINE_UP_RESULT_PERIOD_MS     200  // 結果表示でコマを並べる周期
#define SHOW_RESULT_WAIT_MS          3000 // 結果表示の時間

// ロータリーエンコーダー
#define PULSE_DIFF_PER_CLICK 4 // 1クリックの位相計数
#define UINT16T_MAX 65535      // MTU1.TCNTの最大値...符号なし16ビット

// 74HC595シフトレジスタのシリアルデータ送信コマンド
#define SERIAL_SINK    do { PORT1.PODR.BIT.B5 = 0; } while(0)                        // 吸い込み
#define SERIAL_SOURCE  do { PORT1.PODR.BIT.B5 = 1; } while(0)                        // 吐き出し
#define SEND_LATCH_CLK do { PORT1.PODR.BIT.B6 = 1; PORT1.PODR.BIT.B6 = 0; } while(0) // ラッチ
#define LATCH_OUT      do { PORT1.PODR.BIT.B7 = 1; PORT1.PODR.BIT.B7 = 0; } while(0) // ラッチ出力

// マトリックスLED
#define COL_EN PORTE.PODR.BYTE  // 点灯列許可ビット選択

// 盤面
#define MAT_WIDTH  8 // 横のコマ数
#define MAT_HEIGHT 8 // 縦のコマ数

// リセットボタン オン
#define RESET_BTN_ON (PORTH.PIDR.BIT.B0 == 0)

// 移動オプション
#define MOVE_TYPE_UP_DOWN (PORTH.PIDR.BIT.B3 == 0) // 上下方向移動モード

// AIの先読みの回数
#define AI_DEPTH 3

// 評価関数の重み係数定義. どの要素をどれくらい重要視するか.
#define POS_WEIGHT      7   // 位置評価の重み係数
#define MOBILITY_WEIGHT 3   // 配置可能数評価の重み係数
#define STABLE_WEIGHT   30  // 確定石数（４つ角）評価の重み係数

// 無限大の代わりに使用する大きな値
#define INF 100000
/********************************************************************************************/


/********************************************* 定数 *************************************************/
// 置き判定の時の8方向の移動量
//                        　　　　上       下       左       右      左上      左下     右上     右下
static const int DXDY[8][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}, {-1, 1}, {-1, -1}, {1, 1}, {1, -1}};

// KEY = C majスケール
static const unsigned int C_SCALE[MAT_HEIGHT] = {DO1, RE1, MI1, FA1, SO1, RA1, SI1, DO2};

// 盤面のスコア定義
static const int POSITION_WEIGHTS[MAT_HEIGHT][MAT_WIDTH] =
{
    {120, -40,  20,  10,  10,  20, -40, 120},
    {-40, -50,  -5,  -5,  -5,  -5, -50, -40},
    { 20,  -5,  15,  10,  10,  15,  -5,  20},
    { 10,  -5,  10,   5,   5,  10,  -5,  10},
    { 10,  -5,  10,   5,   5,  10,  -5,  10},
    { 20,  -5,  15,  10,  10,  15,  -5,  20},
    {-40, -50,  -5,  -5,  -5,  -5, -50, -40},
    {120, -40,  20,  10,  10,  20, -40, 120}
};
/*******************************************************************************************/


/**************************************** 型定義 ********************************************/
// オセロの状態管理
enum State {
    // 初期化フェーズ
    INIT_HW,
    INIT_GAME,

	SELECT_VS,
	SELECT_WAIT,

    // ターン開始フェーズ
    TURN_START,
    TURN_CHECK,

    // AI思考フェーズ
    AI_THINK,

    // 入力フェーズ
    INPUT_WAIT,
    INPUT_READ,

    // AI移動フェーズ
    AI_MOVE,

    // 配置フェーズ
    PLACE_CHECK,
    PLACE_OK,
    PLACE_NG,

    // 反転フェーズ
    FLIP_CALC,
    FLIP_RUN,

    // ターン終了フェーズ
    TURN_SWITCH,
    TURN_COUNT,
    TURN_JUDGE,
    TURN_SHOW,

    // ゲーム終了フェーズ
    END_CALC,
    END_SHOW,
    END_WAIT,
    END_RESET
};

// コマが動く方角
enum Direction{
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// マトリックスLEDの色
enum stone_color{
    stone_red,  // 赤コマ
    stone_green,// 緑コマ
    stone_black // 何も置かれていない
};

// ロータリーエンコーダー
struct Rotary{
    unsigned int current_cnt; // 現在のカウント数を保持
    unsigned int prev_cnt;    // 過去のカウント数を保持
    int dir;
};

// カーソル
struct Cursor{
    int x;                  // x座標
    int y;                  // y座標
    int dest_x;             // 目的地のx座標
    int dest_y;             // 目的地のy座標
    enum stone_color color; // カーソルの色
};

// プレイヤー情報
struct Player{
	int placeable_count; // 配置可能数
	int result;          // 最終的なコマの保有数
};

// ゲーム状態
struct Game{
	int count_to_reset;   // リセットボタンのカウント数
	int is_buzzer_active; // サウンドはオンかオフか？
	int is_vs_AI;         // AI対戦モードか？
	int is_AI_turn;       // AIのターンか？
	int is_skip;          // スキップか？
};

// 手の情報を保持する. AI推論用
struct Move{
    int x;     // x座標
    int y;     // y座標
    int score; // 手のスコア
};
/****************************************************************************************/


/************************************* 割り込み使用グローバル変数 ********************************************/
static volatile unsigned long tc_1ms;                                    // 1msタイマーカウンター
static volatile unsigned long tc_2ms;                                    // 2msタイマーカウンター
static volatile unsigned long tc_10ms;                                   // 10msタイマーカウンター
static volatile unsigned long tc_IRQ;                                    // IRQ発生時のタイマカウンター
static volatile unsigned char IRQ1_flag;                                 // IRQ1発生フラグ(sw7)
static volatile unsigned int  beep_period_ms;                            // ブザーを鳴らす時間(1ms基準)
static volatile enum          stone_color screen[MAT_HEIGHT][MAT_WIDTH]; // 割り込みで描画に使用.
static volatile struct        Game *Game_inst_ISR;                       // ISR用Gameインスタンス. IRQ0で使用.
static volatile struct        Cursor cursor;                             // Cursorインスタンス
/************************************************************************************************************/


/************************************************** AI推論用グローバル変数 **************************************************/
// グローバル静的バッファ
static enum        stone_color ai_buf[AI_DEPTH + 1][MAT_HEIGHT][MAT_WIDTH];  // 深さごとのシミュレーションバッファ
static int         ai_entry_idx[MAT_HEIGHT * MAT_WIDTH];                     // ソートに対応させるための座標配列のインデックス
static struct Move ai_moves[AI_DEPTH][MAT_HEIGHT * MAT_WIDTH];               // 各深さでの候補手リスト
static int         ai_move_counts[AI_DEPTH];                                 // 各深さでの候補手数
/***************************************************************************************************************************/


/************************************************** 関数定義 **************************************************/
/********************************************** ハードウェア初期化 *********************************************/
void init_PORT(void)
{
    PORTH.PDR.BIT.B0 = 0;
    PORTH.PDR.BIT.B3 = 0;
    PORT1.PDR.BYTE = 0xE0;
    PORTE.PDR.BYTE = 0xFF;
}

void init_CLK(void)
{
    unsigned int i;
    SYSTEM.PRCR.WORD = 0xA50F;
    SYSTEM.VRCR = 0x00;
    SYSTEM.SOSCCR.BIT.SOSTP = 1;
    while (SYSTEM.SOSCCR.BIT.SOSTP != 1)
        ;
    RTC.RCR3.BYTE = 0x0C;
    while (RTC.RCR3.BIT.RTCEN != 0)
        ;
    SYSTEM.MOFCR.BYTE = 0x0D;
    SYSTEM.MOSCWTCR.BYTE = 0x0D;
    SYSTEM.MOSCCR.BIT.MOSTP = 0x00;
    while (0x00 != SYSTEM.MOSCCR.BIT.MOSTP)
        ;
    for (i = 0; i < 100; i++)
        nop();
    SYSTEM.PLLCR.WORD = 0x0901;
    SYSTEM.PLLWTCR.BYTE = 0x09;
    SYSTEM.PLLCR2.BYTE = 0x00;
    for (i = 0; i < 100; i++)
        nop();
    SYSTEM.OPCCR.BYTE = 0x00;
    while (0 != SYSTEM.OPCCR.BIT.OPCMTSF)
        ;
    SYSTEM.SCKCR.LONG = 0x21821211;
    while (0x21821211 != SYSTEM.SCKCR.LONG)
        ;
    SYSTEM.SCKCR3.WORD = 0x0400;
    while (0x0400 != SYSTEM.SCKCR3.WORD)
        ;
    SYSTEM.PRCR.WORD = 0xA500;
}

void init_CMT0(void)
{
    SYSTEM.PRCR.WORD = 0x0A502;
    MSTP(CMT0) = 0;
    SYSTEM.PRCR.WORD = 0x0A500;
    CMT0.CMCOR = 25000 / 8 - 1;
    CMT0.CMCR.WORD |= 0x00C0;
    IEN(CMT0, CMI0) = 1;
    IPR(CMT0, CMI0) = 1;
    CMT.CMSTR0.BIT.STR0 = 1;
}

void init_CMT1(void)
{
    SYSTEM.PRCR.WORD = 0x0A502;
    MSTP(CMT1) = 0;
    SYSTEM.PRCR.WORD = 0x0A500;
    CMT1.CMCOR = (25000 * 2) / 8 - 1;
    CMT1.CMCR.WORD |= 0x00C0;
    IEN(CMT1, CMI1) = 1;
    IPR(CMT1, CMI1) = 1;
    CMT.CMSTR0.BIT.STR1 = 1;
}

void init_CMT2(void)
{
    SYSTEM.PRCR.WORD = 0x0A502;
    MSTP(CMT2) = 0;
    SYSTEM.PRCR.WORD = 0x0A500;
    CMT2.CMCOR = (25000 * 10) / 8 - 1;
    CMT2.CMCR.WORD |= 0x00C0;
    IEN(CMT2, CMI2) = 1;
    IPR(CMT2, CMI2) = 1;
    CMT.CMSTR1.BIT.STR2 = 1;
}

void init_IRQ0(void)
{
	IEN(ICU, IRQ0) = 0;
	ICU.IRQFLTE0.BIT.FLTEN0 = 0;
	ICU.IRQFLTC0.BIT.FCLKSEL0 = 3;
	PORTH.PDR.BIT.B1 = 0;
	PORTH.PMR.BIT.B1 = 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PH1PFS.BIT.ISEL = 1;
	ICU.IRQCR[0].BIT.IRQMD = 1;
	ICU.IRQFLTE0.BIT.FLTEN0 = 1;
	IR(ICU, IRQ0) = 0;
	IEN(ICU, IRQ0) = 1;
	IPR(ICU, IRQ0) = 1;
}

void init_IRQ1(void)
{
	IEN(ICU, IRQ1) = 0;
	ICU.IRQFLTE0.BIT.FLTEN1 = 0;
	ICU.IRQFLTC0.BIT.FCLKSEL1 = 3;
	PORTH.PDR.BIT.B2 = 0;
	PORTH.PMR.BIT.B2 = 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.PH2PFS.BIT.ISEL = 1;
	ICU.IRQCR[1].BIT.IRQMD  = 1;//立下り
	ICU.IRQFLTE0.BIT.FLTEN1 = 1;
	IR(ICU, IRQ1) = 0;
	IEN(ICU, IRQ1) = 1;
	IPR(ICU, IRQ1) = 1;
}

void init_BUZZER(void)
{
    SYSTEM.PRCR.WORD = 0x0A502;
    MSTP(MTU0) = 0;
    SYSTEM.PRCR.WORD = 0x0A500;
    PORT3.PDR.BIT.B4 = 1;
    PORT3.PMR.BIT.B4 = 1;
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P34PFS.BIT.PSEL = 1;
    MPC.PWPR.BIT.PFSWE = 0;
    MTU.TSTR.BIT.CST0 = 0x00;
    MTU0.TCR.BIT.TPSC = 0x01;
    MTU0.TCR.BIT.CCLR = 0x01;
    MTU0.TMDR.BIT.MD = 0x02;
    MTU0.TIORH.BIT.IOA = 0x06;
    MTU0.TIORH.BIT.IOB = 0x05;
    MTU0.TCNT = 0;
}

void init_MTU1(void)
{
	SYSTEM.PRCR.WORD = 0x0A502;
	MSTP(MTU1) = 0;
	SYSTEM.PRCR.WORD = 0X0A500;

	PORT2.PMR.BIT.B4 = 1;
	PORT2.PMR.BIT.B5 = 1;
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P24PFS.BIT.PSEL = 2;
	MPC.P25PFS.BIT.PSEL = 2;
	MPC.PWPR.BIT.PFSWE = 0;

	MTU1.TMDR.BIT.MD = 4;
	MTU1.TCNT = 0;
	MTU.TSTR.BIT.CST1 = 1;
}

void init_AD0(void)
{
    SYSTEM.PRCR.WORD = 0xA502;
    MSTP(S12AD) = 0;
    SYSTEM.PRCR.WORD = 0xA500;
    PORT4.PMR.BIT.B0 = 1;
    S12AD.ADCSR.BIT.ADIE = 0;
    S12AD.ADANSA.BIT.ANSA0 = 1;
    S12AD.ADCSR.BIT.ADCS = 0;
    MPC.PWPR.BIT.B0WI = 0;
    MPC.PWPR.BIT.PFSWE = 1;
    MPC.P40PFS.BIT.ASEL = 1;
    MPC.PWPR.BIT.PFSWE = 0;
}

void init_RX210(void)
{
    init_CLK();
    init_LCD();
    init_PORT();
    init_CMT0();
    init_CMT1();
    init_CMT2();
    init_IRQ0();
    init_IRQ1();
    init_BUZZER();
    init_MTU1();
    init_AD0();
    setpsw_i();
}
/***********************************************************************************/
/*********************************** ブザー ******************************************/
void beep(unsigned int tone, unsigned int interval, int active)
{
    if (tone && active)
    {
        MTU.TSTR.BIT.CST0 = 0;
        MTU0.TGRA = tone;
        MTU0.TGRB = tone / 2;
        MTU.TSTR.BIT.CST0 = 1;
    }
    else
    {
        MTU.TSTR.BIT.CST0 = 0;
    }

    beep_period_ms = interval;
}

/********************************* LCD表示 ******************************************/
void lcd_show_whose_turn(enum stone_color sc)
{
    lcd_xy(1, 2);
    lcd_puts("                ");
    lcd_xy(1, 2);
    lcd_puts("TURN : ");
    lcd_puts((sc == stone_red) ? "RED" : "GREEN");
    flush_lcd();
}

void lcd_show_skip_msg(void)
{
    lcd_xy(1, 2);
    lcd_puts("                ");
    lcd_xy(1, 2);
    lcd_puts("SKIP PUSH SW7");
    flush_lcd();
}

void lcd_show_winner(int red_stone_count, int green_stone_count)
{
    char *winner;

    lcd_xy(2, 2);

    if(red_stone_count > green_stone_count)
    {
        winner = "RED!";
    }
    else if(red_stone_count < green_stone_count)
    {
        winner = "GREEN!";
    }
    else
    {
        winner = "RED & GREEN!";
    }

    lcd_puts(winner);

    flush_lcd();
}

void lcd_show_confirm(void)
{
    lcd_clear();
    lcd_xy(5, 1);
    lcd_puts("othello");
    lcd_xy(1, 2);
    lcd_puts("NEW -> PUSH SW7");
    flush_lcd();
}
/*************************************************************************************/


/********************************** マトリックスLED ************************************/
// 指定した列の赤緑データをマトリックスLEDに出力
void col_out(int col, unsigned int rg_data)
{
	int i;

    for(i = 0; i < MAT_WIDTH * 2; i++)
	{
		if(rg_data & (1 << i))
		{
			SERIAL_SINK;    // 点灯(カソード側吸い込み)
		}
		else
		{
			SERIAL_SOURCE;  // 消灯(カソード側吐き出し)
		}

		SEND_LATCH_CLK;     // ラッチ
	}

	COL_EN = 0;             // 全消灯

	LATCH_OUT;              // ラッチ出力

	COL_EN = 1 << col;      // 点灯列指定
}
/******************************************************************************************/


/*********************************** ロータリーエンコーダ ***********************************/
// 位相計数用レジスタからカウント数を読み取る
unsigned int read_rotary(void)
{
    return MTU1.TCNT;
}

// アンダーフロー判定
// 位相計数を PULSE_DIFF_PER_CLICK で割って 0～65535/4 にスケーリング
int is_underflow(struct Rotary *r)
{
    return (UINT16T_MAX/PULSE_DIFF_PER_CLICK == r->current_cnt) && (r->prev_cnt == 0);
}

// オーバフロー判定
// 位相計数を PULSE_DIFF_PER_CLICK で割って 0～65535/4 にスケーリング
int is_overflow(struct Rotary *r)
{
    return (UINT16T_MAX/PULSE_DIFF_PER_CLICK == r->prev_cnt) && (r->current_cnt == 0);
}

// 左回りしたか
int is_rotary_turned_left(struct Rotary *r)
{           // カウンター増加　　　　　　　または     オーバーフロー
    return ((r->current_cnt > r->prev_cnt) || is_overflow(r));
}

// 右回りしたか
int is_rotary_turned_right(struct Rotary *r)
{           // カウンター減少              または     アンダーフロー
    return ((r->current_cnt < r->prev_cnt) || is_underflow(r));
}
/**************************************************************************************/


/************************************** コマ/盤面 ********************************************* */
// 何も置かれてないか, または何色が置かれているか
enum stone_color read_stone_at(enum stone_color brd[][MAT_WIDTH], int x, int y)
{
   return brd[y][x];
}

// 指定した色のコマを置く
void place(enum stone_color brd[][MAT_WIDTH], int x, int y, enum stone_color sc)
{
    brd[y][x] = sc;
}

// 指定した座標のコマを消す
void delete(enum stone_color brd[][MAT_WIDTH], int x, int y)
{
    brd[y][x] = stone_black;
}

// ローカルボードの内容を割込み用表示ボードにコピー（フラッシュ）
void flush_board(enum stone_color brd[][MAT_WIDTH])
{
    int x, y;

    for(x = 0; x < MAT_WIDTH; x++)
    {
        for(y = 0; y < MAT_HEIGHT; y++)
        {
            screen[y][x] = brd[y][x];
        }
    }
}
/*****************************************************************************/


/****************************** カーソル **************************************/
// カーソルの座標をセット
void set_cursor_xy(int x, int y)
{
    cursor.x = x;
    cursor.y = y;
}

// カーソルの色をセット
void set_cursor_color(enum stone_color sc)
{
    cursor.color = sc;
}

// 上下左右を指定してカーソルの座標を更新
void move_cursor(enum Direction dir)
{
    int cx = cursor.x;
    int cy = cursor.y;

    if (dir == LEFT)  // 左
    {
        // 左シフト
        cx--;

        // 左端まで行ったら一つ上の行の右端にワープ
        if (cx < 0)
        {
        	cx = MAT_WIDTH - 1;
        	cy++;

            // 上端を超えたら一番下の行にワープ
            if (cy > MAT_HEIGHT - 1)
            {
            	cy = 0;
            }
        }
    }
    else if (dir == RIGHT)  // 右
    {
        // 右シフト
    	cx++;

        // 右端まで行ったら一つ下の行の左端にワープ
        if (cx > MAT_WIDTH - 1)
        {
        	cx = 0;
        	cy--;

            // 下端を超えたら一番上の行にワープ
            if (cy < 0)
            {
            	cy = MAT_HEIGHT - 1;
            }
        }
    }
    else if (dir == UP) // 上
    {
        // 上移動
    	cy++;

        // 上端まで行ったら一つの右の列の下端にワープ
		if (cy > MAT_HEIGHT - 1)
		{
			cx++;
			cy = 0;

            // 右端を超えたら一番左の列にワープ
			if (cx > MAT_WIDTH - 1)
			{
				cx = 0;
			}
		}
    }
    else if(dir == DOWN) // 下
    {
        // 下移動
    	cy--;

        // 下端まで行ったら一つ左の行の上端にワープ
		if (cy < 0)
		{
			cy = MAT_HEIGHT - 1;
			cx--;

            // 左端を超えたら一番右の列にワープ
			if (cx < 0)
			{
				cx = MAT_WIDTH - 1;
			}
		}
    }

    set_cursor_xy(cx, cy);
}
/**********************************************************************************/


/************************************ ゲームロジック *********************************/
// タイミング調整
void wait_10ms(int period)
{
    tc_10ms = 0;
    while(tc_10ms < period)
        nop();
}

// AD変換値を取得. 乱数のシード値に利用.
unsigned int get_AD0_val(void)
{
    S12AD.ADCSR.BIT.ADST = 1;
    while (1 == S12AD.ADCSR.BIT.ADST)
        ;

    return (unsigned int)S12AD.ADDR0;
}

// 座標範囲外か
int is_out_of_board(int x, int y)
{
    return ((x < 0) || (y < 0) || ( x > MAT_WIDTH  - 1) || (y > MAT_HEIGHT - 1));
}

// 8方向のひっくり返しフラグを作る
//　       右下  右上  左下  左上  右   左   下   上
// flag :  b7    b6    b5    b4  b3   b2   b1   b0
// bit  :  0..その方角にひっくり返せない, 1..その方角にひっくり返せる
unsigned char make_flip_dir_flag(enum stone_color brd[][MAT_WIDTH], int x, int y, enum stone_color sc)
{
    int dir, i;
    int dx, dy;
    unsigned char flag = 0x00;

    enum stone_color search;

    for(dir = 0; dir < 8; dir++)
    {
        dx = dy = 0;

        for(i = 0; i < 8; i++)
        {
            dx += DXDY[dir][0];
            dy += DXDY[dir][1];

            // 範囲外ならbreak
            if(is_out_of_board(x + dx, y + dy)) break;

            // コマの色を調査
            search = read_stone_at(brd, x + dx, y + dy);

            // 何も置かれていなかったらbreak
            if(search == stone_black) break;

            // 挟む側のコマの色に遭遇
            if(search == sc)
            {
                // i > 0 の時点で相手色を少なくとも1つは挟んでいる
                if(i > 0)
                {
                    flag |= (1 << dir);
                }

                break;
            }
        }
    }

    return flag;
}

//その場所にその色は置けるか？
int is_placeable(enum stone_color brd[][MAT_WIDTH], int x, int y, enum stone_color sc)
{
    unsigned char flag;

    // 何かおいてあったらだめ
    if(read_stone_at(brd, x, y) != stone_black) return 0;

     // 8方向フラグ作成
    flag = make_flip_dir_flag(brd, x, y, sc);

    // flag != 0x00なら少なくとも1方向は挟める
    return (flag != 0x00);
}

// 8方向フラグをつかって相手のコマをひっくり返す
void flip_stones(unsigned char flag, enum stone_color brd[][MAT_WIDTH], int x, int y, enum stone_color sc)
{
    int dir, i;
    int dx, dy;
    enum stone_color search;

    for(dir = 0; dir < 8; dir++)
    {
        dx = dy = 0;

        if(flag & (1 << dir))
        {
            for(i = 0; i < 8; i++)
            {
                dx += DXDY[dir][0];
                dy += DXDY[dir][1];

                // コマの色をチェック
                search = read_stone_at(brd, x + dx, y + dy);

                // 置きチェック済みなので確認するのは自分の色が出たかのみ
                if(search == sc)
                {
                    break;
                }

                // コマを消す
                delete(brd, x + dx, y + dy);

                // 新しくコマを置く
                place(brd, x + dx, y + dy, (search == stone_red) ? stone_green : stone_red);
            }
        }
    }
}

// ボード上にその色のコマが置ける場所はあるか
int count_placeable(enum stone_color brd[][MAT_WIDTH], enum stone_color sc)
{
    int x, y;
    int count = 0;

    for(x = 0; x < MAT_WIDTH; x++)
    {
        for(y = 0; y < MAT_HEIGHT; y++)
        {
            if(is_placeable(brd, x, y, sc))
            {
                count++;
            }
        }
    }

    return count;
}

// 指定した色のコマの数を数える
int count_stones(enum stone_color brd[][MAT_WIDTH], enum stone_color sc)
{
    int x, y;
    int count = 0;

    for(x = 0; x < MAT_WIDTH; x++)
    {
        for(y = 0; y < MAT_HEIGHT; y++)
        {
            if(read_stone_at(brd, x, y) == sc)
            {
                count++;
            }
        }
    }

    return count;
}

// どっちも置けなかったらおわり
int is_game_over(int stone1_placeable_count, int stone2_placeable_count)
{
    return (!stone1_placeable_count && !stone2_placeable_count);
}

// コマを並べて結果発表
void line_up_result(enum stone_color brd[][MAT_WIDTH], int stone1_count, int stone2_count, int period_10ms, int *buzzer_active)
{
	int x, y;

    // コマを全撤去
	for(x = 0; x < MAT_WIDTH; x++)
	{
		for(y = 0; y < MAT_HEIGHT; y++)
        {
            delete(brd, x, y);
        }
	}

    flush_board(brd);

	x = 0;

    // 最終結果をもとに再配置
	while(stone1_count || stone2_count)
	{
		if(stone1_count)
		{
            // 片方の色を左上から詰めていく
            place(brd, x % MAT_WIDTH, ((MAT_WIDTH - 1) - (x / MAT_WIDTH)), stone_red);

			stone1_count--;
		}
		else
		{   // 詰め終わったら続きからもう片方の色を詰めていく
			place(brd, x % MAT_WIDTH, ((MAT_WIDTH - 1) - (x / MAT_WIDTH)), stone_green);

		    stone2_count--;
		}

        flush_board(brd);

        // x座標に合わせてドレミ
		beep(C_SCALE[x % MAT_WIDTH], 50, *buzzer_active);

        // 詰めの感覚を調整
		wait_10ms(period_10ms);

		x++;
	}
}

/********************************************* AI ***********************************************/
// 盤面の位置評価を計算
int evaluate_position_weight(enum stone_color brd[][MAT_WIDTH], enum stone_color ai_color)
{
    int x, y;
    int ai_score = 0;
    int opp_score = 0;
    enum stone_color opp_color = (ai_color == stone_red) ? stone_green : stone_red;

    for(y = 0; y < MAT_HEIGHT; y++)
    {
        for(x = 0; x < MAT_WIDTH; x++)
        {
            if(brd[y][x] == ai_color)
            {
                ai_score += POSITION_WEIGHTS[y][x];
            }
            else if(brd[y][x] == opp_color)
            {
                opp_score += POSITION_WEIGHTS[y][x];
            }
        }
    }

    return ai_score - opp_score;
}

// コマの数の差を計算. 終盤用.
int evaluate_stone_count(enum stone_color brd[][MAT_WIDTH], enum stone_color ai_color)
{
    int x, y;
    int ai_count = 0;
    int opp_count = 0;
    enum stone_color opp_color = (ai_color == stone_red) ? stone_green : stone_red;

    for(y = 0; y < MAT_HEIGHT; y++)
    {
        for(x = 0; x < MAT_WIDTH; x++)
        {
            if(brd[y][x] == ai_color)
            {
                ai_count++;
            }
            else if(brd[y][x] == opp_color) 
            {
                opp_count++;
            }
        }
    }

    return ai_count - opp_count;
}

// 絶対に取られないコマの数を計算
int count_stable_stones(enum stone_color brd[][MAT_WIDTH], enum stone_color color)
{
    int stable_count = 0;

    // 角のコマは確定石
    if(brd[0][0] == color) stable_count++;
    if(brd[0][MAT_WIDTH-1] == color) stable_count++;
    if(brd[MAT_HEIGHT-1][0] == color) stable_count++;
    if(brd[MAT_HEIGHT-1][MAT_WIDTH-1] == color) stable_count++;

    return stable_count;
}

// 盤面を評価する関数. AI視点でのスコア.
int evaluate_board(enum stone_color brd[][MAT_WIDTH], enum stone_color ai_color)
{
    enum stone_color opp_color = (ai_color == stone_red) ? stone_green : stone_red;
    int position_score, mobility_score, stable_score;
    int ai_stable, opp_stable;

    // 位置評価
    position_score = evaluate_position_weight(brd, ai_color);

    // 配置可能数評価. 相手の手数が少ないほど有利.
    mobility_score = -count_placeable(brd, opp_color);

    // 確定石評価
    ai_stable = count_stable_stones(brd, ai_color);
    opp_stable = count_stable_stones(brd, opp_color);
    stable_score = (ai_stable - opp_stable) * STABLE_WEIGHT;

    return position_score * POS_WEIGHT + mobility_score * MOBILITY_WEIGHT + stable_score;
}

// ミニマックス法 + αβ枝刈り
int minimax_alphabeta(enum stone_color brd[][MAT_WIDTH], enum stone_color ai_color, int max_depth)
{
    int depth, x, y, i, move_idx;
    enum stone_color current_color;
    int score, best_score;
    int is_max_player;

    // スタック用の変数
    int stack_alpha[AI_DEPTH + 1];
    int stack_beta[AI_DEPTH + 1];
    int stack_best_score[AI_DEPTH + 1];
    int stack_move_idx[AI_DEPTH + 1];
    int stack_is_max[AI_DEPTH + 1];

    // 初期化
    memcpy(ai_buf[0], brd, sizeof(enum stone_color) * MAT_HEIGHT * MAT_WIDTH);

    // ルートノードの候補手を生成
    ai_move_counts[0] = 0;
    for(y = 0; y < MAT_HEIGHT; y++)
    {
        for(x = 0; x < MAT_WIDTH; x++)
        {
            if(is_placeable(ai_buf[0], x, y, ai_color))
            {
                ai_moves[0][ai_move_counts[0]].x = x;
                ai_moves[0][ai_move_counts[0]].y = y;
                ai_moves[0][ai_move_counts[0]].score = -INF;
                ai_move_counts[0]++;
            }
        }
    }

    if(ai_move_counts[0] == 0) return -INF;

    best_score = -INF;

    // 各候補手を評価
    for(i = 0; i < ai_move_counts[0]; i++)
    {
        x = ai_moves[0][i].x;
        y = ai_moves[0][i].y;

        // 手を打つ
        memcpy(ai_buf[1], ai_buf[0], sizeof(enum stone_color) * MAT_HEIGHT * MAT_WIDTH);
        flip_stones(make_flip_dir_flag(ai_buf[1], x, y, ai_color), ai_buf[1], x, y, ai_color);

        // 深さ1から探索開始
        depth = 1;
        stack_alpha[1] = -INF;
        stack_beta[1] = INF;
        stack_move_idx[1] = 0;
        stack_is_max[1] = 0;  // 次は相手のターン
        score = -INF;

        while(depth > 0)
        {
            if(depth >= max_depth)
            {
                // 葉ノード：評価値を計算
                score = evaluate_board(ai_buf[depth], ai_color);
                depth--;

                if(depth > 0)
                {
                    if(stack_is_max[depth])
                    {
                        if(score > stack_best_score[depth])
                            stack_best_score[depth] = score;

                        if(stack_best_score[depth] >= stack_beta[depth])
                        {
                            // β枝刈り
                            score = stack_best_score[depth];
                            depth--;
                            if(depth > 0)
                            {
                                stack_move_idx[depth]++;
                            }
                            continue;
                        }

                        if(stack_best_score[depth] > stack_alpha[depth])
                            stack_alpha[depth] = stack_best_score[depth];
                    }
                    else
                    {
                        if(score < stack_best_score[depth])
                            stack_best_score[depth] = score;

                        if(stack_best_score[depth] <= stack_alpha[depth])
                        {
                            // α枝刈り
                            score = stack_best_score[depth];
                            depth--;
                            if(depth > 0)
                            {
                                stack_move_idx[depth]++;
                            }
                            continue;
                        }

                        if(stack_best_score[depth] < stack_beta[depth])
                            stack_beta[depth] = stack_best_score[depth];

                    }
                    stack_move_idx[depth]++;
                }
                continue;
            }

            // 現在のプレイヤー
            is_max_player = stack_is_max[depth];
            current_color = (depth % 2 == 1) ? (ai_color == stone_red ? stone_green : stone_red) : ai_color;

            // 初回訪問時：候補手を生成
            if(stack_move_idx[depth] == 0)
            {
                ai_move_counts[depth] = 0;
                for(y = 0; y < MAT_HEIGHT; y++)
                {
                    for(x = 0; x < MAT_WIDTH; x++)
                    {
                        if(is_placeable(ai_buf[depth], x, y, current_color))
                        {
                            ai_moves[depth][ai_move_counts[depth]].x = x;
                            ai_moves[depth][ai_move_counts[depth]].y = y;
                            ai_move_counts[depth]++;
                        }
                    }
                }

                // 手がない場合
                if(ai_move_counts[depth] == 0)
                {
                    // パス：評価値を返す
                    score = evaluate_board(ai_buf[depth], ai_color);
                    depth--;

                    if(depth > 0)
                    {
                        if(stack_is_max[depth])
                        {
                            if(score > stack_best_score[depth])
                                stack_best_score[depth] = score;

                        }
                        else
                        {
                            if(score < stack_best_score[depth])
                                stack_best_score[depth] = score;
                        }

                        stack_move_idx[depth]++;
                    }
                    continue;
                }

                stack_best_score[depth] = is_max_player ? -INF : INF;
            }

            // すべての手を評価済み
            if(stack_move_idx[depth] >= ai_move_counts[depth])
            {
                score = stack_best_score[depth];
                depth--;

                if(depth > 0)
                {
                    if(stack_is_max[depth])
                    {
                        if(score > stack_best_score[depth])
                            stack_best_score[depth] = score;

                        if(stack_best_score[depth] >= stack_beta[depth])
                        {
                            // β枝刈り
                            score = stack_best_score[depth];
                            depth--;
                            if(depth > 0)
                            {
                                stack_move_idx[depth]++;
                            }
                            continue;
                        }

                        if(stack_best_score[depth] > stack_alpha[depth])
                            stack_alpha[depth] = stack_best_score[depth];
                    }
                    else
                    {
                        if(score < stack_best_score[depth])
                            stack_best_score[depth] = score;

                        if(stack_best_score[depth] <= stack_alpha[depth])
                        {
                            // α枝刈り
                            score = stack_best_score[depth];
                            depth--;
                            if(depth > 0)
                            {
                                stack_move_idx[depth]++;
                            }
                            continue;
                        }

                        if(stack_best_score[depth] < stack_beta[depth])
                            stack_beta[depth] = stack_best_score[depth];
                    }

                    stack_move_idx[depth]++;
                }
                continue;
            }

            // 次の手を試す
            move_idx = stack_move_idx[depth];
            x = ai_moves[depth][move_idx].x;
            y = ai_moves[depth][move_idx].y;

            // 手を打つ
            memcpy(ai_buf[depth + 1], ai_buf[depth], sizeof(enum stone_color) * MAT_HEIGHT * MAT_WIDTH);
            flip_stones(make_flip_dir_flag(ai_buf[depth + 1], x, y, current_color), ai_buf[depth + 1], x, y, current_color);

            // 次の深さへ
            depth++;
            stack_alpha[depth] = stack_alpha[depth - 1];
            stack_beta[depth] = stack_beta[depth - 1];
            stack_move_idx[depth] = 0;
            stack_is_max[depth] = !is_max_player;
        }

        ai_moves[0][i].score = score;
        if(score > best_score)
        {
            best_score = score;
        }
    }

    return best_score;
}

// AIの次の行き先を決める
void set_AI_cursor_dest(enum stone_color brd[][MAT_WIDTH], enum stone_color sc, int placeable_count, int depth)
{
    int i, best_idx, best_count;
    int best_score;

    // スキップ = どこにも置けない場合は現在のカーソル位置を返す
    if(!placeable_count)
    {
        cursor.dest_x = cursor.x;
        cursor.dest_y = cursor.y;
        return;
    }

    // ミニマックス + αβ枝刈りで評価
    minimax_alphabeta(brd, sc, depth);

    // 最高評価の手を見つける
    best_score = -INF;

    for(i = 0; i < ai_move_counts[0]; i++)
    {
        if(ai_moves[0][i].score > best_score)
        {
            best_score = ai_moves[0][i].score;
        }
    }

    // 同じスコアの手の数をカウント
    best_count = 0;

    for(i = 0; i < ai_move_counts[0]; i++)
    {
        if(ai_moves[0][i].score == best_score)
        {
            ai_entry_idx[best_count] = i;
            best_count++;
        }
    }

    // 同点の場合はランダムに選択
    if(best_count > 1)
    {
        best_idx = ai_entry_idx[rand() % best_count];
    }
    else
    {
        best_idx = ai_entry_idx[0];
    }

    cursor.dest_x = ai_moves[0][best_idx].x;
    cursor.dest_y = ai_moves[0][best_idx].y;
}
/*************************************************************************************************/


/***************************************** 初期設定 ******************************************/
// 位相計数用レジスタのカウント数初期化
void clear_pulse_diff_cnt(void)
{
    MTU1.TCNT = 0;
}

// ロータリーエンコーダー情報初期化
void init_Rotary(struct Rotary *r)
{
    r->current_cnt = 0;
    r->prev_cnt    = 0;
}

// ゲーム情報初期化
void init_Game(struct Game *g)
{
	g->count_to_reset   = 0;
	g->is_buzzer_active = 1;
	g->is_vs_AI         = 0;
	g->is_AI_turn       = 0;
	g->is_skip          = 0;
}

// プレイヤー情報初期化
void init_Player(struct Player *p1, struct Player *p2)
{
    // オセロのルール上最初は二か所しか置けない
	p1->placeable_count = 2;
	p2->placeable_count = 2;
    p1->result          = 0;
	p2->result          = 0;
}

// 盤面初期化
void init_board(enum stone_color brd[][MAT_WIDTH])
{   int x, y;

    // コマ全撤去
    for(x = 0;x < MAT_WIDTH; x++)
    {
        for(y = 0; y < MAT_HEIGHT; y++)
        {
            delete(brd, x, y);
        }
    }

    // 真ん中に４つ置く
    place(brd, 3, 3, stone_red);
    place(brd, 4, 4, stone_red);
    place(brd, 3, 4, stone_green);
    place(brd, 4, 3, stone_green);
}

// カーソル初期化
void init_Cursor(int x, int y, enum stone_color sc)
{
    set_cursor_color(sc);
    set_cursor_xy(x, y);
}

// LCD表示初期化
void init_lcd_show(enum stone_color sc)
{
	lcd_clear();
	lcd_xy(5, 1);
	lcd_puts("othello");
	lcd_xy(1, 2);
	lcd_puts("VS >FRIEND : AI");
	flush_lcd();
}
/*************************************************************************************************/


/****************************************** 割込み ************************************************/
// CMT0 CMI0 1msタイマ割込み
void Excep_CMT0_CMI0(void)
{
	tc_1ms++;

	beep_period_ms--;

    // 指定時間たったら音を止める
	if(!beep_period_ms)
	{
		MTU.TSTR.BIT.CST0 = 0;
	}
}

// CMT1 CMI1 2msタイマ割込み
void Excep_CMT1_CMI1(void)
{
	int x, y;
    unsigned int rg_data = 0x0000;

    tc_2ms++;

	x = tc_2ms % MAT_WIDTH;

    for(y = 0; y < MAT_HEIGHT; y++)
    {
        if(screen[y][x] == stone_red)
        {
            rg_data |= (1 << (y + 8));
        }
        else if(screen[y][x] == stone_green)
        {
            rg_data |= (1 << y);
        }
    }

    if((x != cursor.x) || (cursor.color == stone_black))
    {
        // マトリックスLED出力
        col_out(x, rg_data);

        return;
    }

    // 一定間隔でカーソルを点滅させる
    if((tc_2ms / (CURSOR_BLINK_PERIOD_MS / 2)) % 2)
    {
        rg_data |= (cursor.color == stone_red) ? (1 << (cursor.y + 8)) : (1 << cursor.y);
    }
    else if(rg_data & ((1 << (cursor.y + 8)) | (1 << cursor.y)))
    {
        rg_data &= ~((1 << (cursor.y + 8)) | (1 << cursor.y));
    }

    //マトリックスLED出力
    col_out(x, rg_data);
}

// CMT2 CMI2 10msタイマ割込み
void Excep_CMT2_CMI2(void)
{
	tc_10ms++;
}

// ICU IRQ0 SW6立下がり割込み
void Excep_ICU_IRQ0(void)
{
	unsigned long now = tc_1ms;

	// 前のIRQ発生から指定時間経ってなかったらreturn
	if(now - tc_IRQ < MONITOR_CHATTERING_PERIOD_MS) return;

    Game_inst_ISR->is_buzzer_active ^= 1;

	tc_IRQ = now;
}

// ICU IRQ1 SW7立下がり割込み
void Excep_ICU_IRQ1(void)
{
	unsigned long now = tc_1ms;

    // 前のIRQ発生から指定時間経ってなかったらreturn
	if(now - tc_IRQ < MONITOR_CHATTERING_PERIOD_MS) return;

    IRQ1_flag = 1;

    tc_IRQ = now;
}
/**************************************************************************************************/
/******************************************* 関数定義終 ********************************************/


/******************************************** メイン ***********************************************/
void main(void)
{
    // 状態管理
    enum State state = INIT_HW;

    // ボード色情報
    enum stone_color board[MAT_HEIGHT][MAT_WIDTH];

	// ゲーム情報
    struct Game game;

    // 赤緑プレイヤー
    struct Player red, green;

    // ロータリーエンコーダ入力
    struct Rotary rotary;

    // コマ反転用フラグ
	//　       右下  右上  左下  左上  右   左   下   上
	// flag :  b7    b6    b5    b4  b3   b2   b1   b0
	// bit  :  0..その方角にひっくり返せない, 1..その方角にひっくり返せる
    unsigned char flip_dir_flag;

    // 経過時間計測スタート
    unsigned long start_tc = tc_1ms;

    // 割り込み(IRQ0)用インスタンス
    Game_inst_ISR = &game;

    init_RX210();

    while(1)
    {
		// リセットボタン(sw5)を1秒間隔で監視
    	if(tc_1ms - start_tc > 1000)
    	{
    		if(RESET_BTN_ON)
			{
    			beep(DO1, 50, game.is_buzzer_active);
				game.count_to_reset++;
			}
			else
			{
				game.count_to_reset = 0;
			}

            // 2～3秒長押しされたらリセット
    		if(game.count_to_reset > 2)
    		{
    			beep(DO2, 300, game.is_buzzer_active);
    			state = INIT_HW;
    		}

    		start_tc = tc_1ms;
    	}

        switch(state)
        {
            //********** 初期化フェーズ **********//
            case INIT_HW:

                clear_pulse_diff_cnt();
                init_Rotary(&rotary);
                state = INIT_GAME;
                break;

            case INIT_GAME:

                srand(get_AD0_val());
                init_Game(&game);
                init_Player(&red, &green);
                init_board(board);
                init_Cursor(5, 3, stone_red);
                init_lcd_show(cursor.color);
                flush_board(board);
                state = SELECT_WAIT;
                break;

            case SELECT_WAIT:

                if(IRQ1_flag)
                {
                	beep(DO2, 200, game.is_buzzer_active);
                    lcd_show_whose_turn(cursor.color);
                    state = TURN_START;
                    IRQ1_flag = 0;
                }
                else
                {
                	state = SELECT_VS;
                }

                break;

            case SELECT_VS:

				rotary.current_cnt = read_rotary() / PULSE_DIFF_PER_CLICK;

				if(rotary.current_cnt != rotary.prev_cnt)
				{
					beep(DO3, 50, game.is_buzzer_active);
			   		game.is_vs_AI ^= 1;

                    if(game.is_vs_AI)
				    {
                        lcd_xy(1, 2);
                        lcd_puts("VS  FRIEND :>AI");
					    flush_lcd();
				    }
				    else
				    {
					    lcd_xy(1, 2);
                        lcd_puts("VS >FRIEND : AI");
					    flush_lcd();
				    }
				}

				rotary.prev_cnt = rotary.current_cnt;

				state = SELECT_WAIT;

            	break;

            //********** ターン開始フェーズ **********//
            case TURN_START:

                state = TURN_CHECK;
                break;

            case TURN_CHECK:

                if(game.is_AI_turn && game.is_vs_AI)
                {
                    state = AI_THINK;
                }
                else
                {
                    state = INPUT_WAIT;
                }

                break;

            //********** AI思考フェーズ **********//
            case AI_THINK:

                set_AI_cursor_dest(board, cursor.color, (cursor.color == stone_red) ? red.placeable_count : green.placeable_count, AI_DEPTH);
                state = AI_MOVE;
                break;

            //********** プレイヤー入力フェーズ **********//
            case INPUT_WAIT:

                if(IRQ1_flag)
                {
                    state = PLACE_CHECK;
                    IRQ1_flag = 0;
                }
                else
                {
                    state = INPUT_READ;
                }

                break;

            case INPUT_READ:

                rotary.current_cnt = read_rotary() / PULSE_DIFF_PER_CLICK;

                if(is_rotary_turned_left(&rotary))
                {
                    move_cursor((MOVE_TYPE_UP_DOWN) ? DOWN : LEFT);
                    beep(C_SCALE[(MOVE_TYPE_UP_DOWN) ? cursor.y : cursor.x], 100, game.is_buzzer_active);
                }
                else if(is_rotary_turned_right(&rotary))
                {
                    move_cursor((MOVE_TYPE_UP_DOWN) ? UP : RIGHT);
                    beep(C_SCALE[(MOVE_TYPE_UP_DOWN) ? cursor.y : cursor.x], 100, game.is_buzzer_active);
                }

                rotary.prev_cnt = rotary.current_cnt;

                state = INPUT_WAIT;

                break;

            //********** AI自動移動フェーズ **********//
            case AI_MOVE:

                if(cursor.x < cursor.dest_x)
                {
                    beep(C_SCALE[cursor.x], 100, game.is_buzzer_active);
                    move_cursor(RIGHT);
                }
                else if(cursor.x > cursor.dest_x)
                {
                    beep(C_SCALE[cursor.x], 100, game.is_buzzer_active);
                    move_cursor(LEFT);
                }

                if(cursor.y < cursor.dest_y)
                {
                    beep(C_SCALE[cursor.y], 100, game.is_buzzer_active);
                    move_cursor(UP);
                }
                else if(cursor.y > cursor.dest_y)
                {
                    beep(C_SCALE[cursor.y], 100, game.is_buzzer_active);
                    move_cursor(DOWN);
                }

                if((cursor.x == cursor.dest_x) && (cursor.y == cursor.dest_y))
                {
                    state = PLACE_CHECK;
                }

                wait_10ms(AI_MOVE_PERIOD_MS / 10);
                break;

            //********** コマ配置フェーズ **********//
            case PLACE_CHECK:

                if(game.is_skip)
                {
                    // スキップの場合は配置せずにターン終了
                    state = TURN_SWITCH;
                }
                else if(is_placeable(board, cursor.x, cursor.y, cursor.color))
                {
                    state = PLACE_OK;
                }
                else
                {
                    state = PLACE_NG;
                }

                break;

            case PLACE_OK:

            	beep(DO2, 100, game.is_buzzer_active);
                place(board, cursor.x, cursor.y, cursor.color);
                flush_board(board);
                state = FLIP_CALC;
                break;

            case PLACE_NG:

                beep(DO0, 100, game.is_buzzer_active);
                // プレイヤーの場合は入力待ちに戻る、AIの場合は理論上ここに来ない
                state = (game.is_AI_turn) ? TURN_START : INPUT_WAIT;
                break;

            //********** コマ反転フェーズ **********//
            case FLIP_CALC:

                flip_dir_flag = make_flip_dir_flag(board, cursor.x, cursor.y, cursor.color);
                state = FLIP_RUN;
                break;

            case FLIP_RUN:

                flip_stones(flip_dir_flag, board, cursor.x, cursor.y, cursor.color);
                flush_board(board);
                state = TURN_SWITCH;
                break;

            //********** ターン終了フェーズ **********//
            case TURN_SWITCH:

            	cursor.color = ((cursor.color == stone_red) ? stone_green : stone_red);
                state = TURN_COUNT;
                break;

            case TURN_COUNT:

                red.placeable_count   = count_placeable(board, stone_red);
                green.placeable_count = count_placeable(board, stone_green);
                state = TURN_JUDGE;
                break;

            case TURN_JUDGE:

                if(is_game_over(red.placeable_count, green.placeable_count))
                {
                    state = END_CALC;
                }
                else
                {
                    game.is_skip = (cursor.color == stone_red) ? !red.placeable_count : !green.placeable_count;
                    state = TURN_SHOW;
                }

                break;

            case TURN_SHOW:

                if(game.is_skip)
                {
                    lcd_show_skip_msg();
                }
                else
                {
                    lcd_show_whose_turn(cursor.color);
                }

                // AIプレイヤーの切り替え
                if(game.is_vs_AI) game.is_AI_turn ^= 1;

                state = TURN_START;
                break;

            //********** ゲーム終了フェーズ **********//
            case END_CALC:

                red.result   = count_stones(board, stone_red);
                green.result = count_stones(board, stone_green);
                state = END_SHOW;
                break;

            case END_SHOW:

                lcd_clear();
                lcd_puts("Winner is ...");
                flush_lcd();

                set_cursor_color(stone_black);
                line_up_result(board, red.result, green.result, LINE_UP_RESULT_PERIOD_MS / 10, &game.is_buzzer_active);

                lcd_show_winner(red.result, green.result);

                wait_10ms(SHOW_RESULT_WAIT_MS / 10);

                lcd_show_confirm();
                state = END_WAIT;
                break;

            case END_WAIT:

                if(IRQ1_flag)
                {
                    IRQ1_flag = 0;
                    state = END_RESET;
                }

                break;

            case END_RESET:

                state = INIT_HW;
                break;

            default:
                break;
        }
    }
}

#ifdef __cplusplus
extern "C" void abort(void) {}
#endif

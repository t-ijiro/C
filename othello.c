/***********************************************************************/
/*                                                                     */
/*  FILE        : othello.c                                            */
/*  DATE        : 2025/11/16 SUN                                       */
/*  DESCRIPTION : Main Program                                         */
/*  CPU TYPE    : RX Family                                            */
/*                                                                     */
/*  Author t.ijiro                                                     */
/*                                                                     */
/***********************************************************************/
// #include "typedefine.h"
#ifdef __cplusplus
// #include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

#include <machine.h>
#include "lcd_lib4.h"
#include "iodefine.h"
#include "vect.h"

/*************************** マクロ ****************************/
//ロータリーエンコーダー
#define PULSE_DIFF_PER_CLICK 4 //1クリックの位相計数
#define UINT16T_MAX 65535      //MTU1.TCNTの最大値...符号なし16ビット

// 74HC595シフトレジスタのシリアルデータ送信コマンド
#define SERIAL_SINK    do { PORT1.PODR.BIT.B5 = 0; } while(0)                        // 吸い込み
#define SERIAL_SOURCE  do { PORT1.PODR.BIT.B5 = 1; } while(0)                        // 吐き出し
#define SEND_LATCH_CLK do { PORT1.PODR.BIT.B6 = 1; PORT1.PODR.BIT.B6 = 0; } while(0) // ラッチ
#define LATCH_OUT      do { PORT1.PODR.BIT.B7 = 1; PORT1.PODR.BIT.B7 = 0; } while(0) // ラッチ出力

//マトリックスLED
#define COL_EN PORTE.PODR.BYTE  //点灯列許可ビット選択

//盤面
#define MAT_WIDTH  8 //横のコマ数
#define MAT_HEIGHT 8 //縦のコマ数

//移動オプション
#define MOVE_TYPE_UP_DOWN (PORT5.PIDR.BIT.B0 == 0) //上下方向移動モード
/**************************************************************/


/**************************** 定数 ******************************/
//置き判定の時の8方向の移動量 
//                        上       下       左       右      左上      左下     右上     右下
const int dxdy[8][2] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}, {-1, 1}, {-1, -1}, {1, 1}, {1, -1}};
/****************************************************************/


/*************************** 型定義 ****************************/
//オセロの状態管理
enum State{
    INIT,
    MOVE,
    PLACE,
    FLIP,
    TURN_OVER,
    GAME_OVER
};

//コマが動く方角
enum Direction{
    LEFT,
    RIGHT,
    UP,
    DOWN
};

//マトリックスLEDの色
enum stone_color{
    stone_red,  //赤コマ
    stone_green,//緑コマ
    stone_black //何も置かれていない
};

//ロータリーエンコーダー
struct Rotary{
    unsigned int current_cnt; //現在のカウント数を保持
    unsigned int prev_cnt;    //過去のカウント数を保持
};

//コマ
struct Stone{
    unsigned char stone[MAT_HEIGHT]; //各列のビットマスクでコマを表現
    int can_place;                   //置けるところはあるか？
};

//カーソル
struct Cursor{
    int x;                  //x座標
    int y;                  //y座標
    enum stone_color color; //カーソルの色
};
/****************************************************************************************/


/************************************* グローバル変数 *************************************/
volatile unsigned long tc_1ms;           //1msタイマーカウンター
volatile unsigned char sw7_flag;         //sw7割り込み発生フラグ(IRQ1)
volatile struct Stone red, green;        //Stoneインスタンス
volatile struct Cursor cursor;           //Cursorインスタンス
volatile unsigned int screen[MAT_WIDTH]; //描画データ. 上位8ビットに赤, 下位8ビットに緑のstoneデータを格納する.
/*************************************************************************************/


/************************************** 関数定義 *****************************************/
/********************************** ハードウェア初期化 ***********************************/
void init_PORT(void)
{
    PORT5.PDR.BIT.B0 = 0;
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

void init_CMT1(void)
{
    SYSTEM.PRCR.WORD = 0x0A502;
    MSTP(CMT1) = 0;
    SYSTEM.PRCR.WORD = 0x0A500;
    CMT1.CMCOR = 25000 / 8 - 1;
    CMT1.CMCR.WORD |= 0x00C0;
    IEN(CMT1, CMI1) = 1;
    IPR(CMT1, CMI1) = 1;
    CMT.CMSTR0.BIT.STR1 = 1;
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

void init_HARDWARE(void)
{
    init_CLK(); 
    init_LCD();
    init_PORT();
    init_CMT1();
    init_IRQ1();
    init_MTU1();
    setpsw_i();
}
/***********************************************************************************/


/********************************* LCD表示 ******************************************/
void init_display(void)
{
  lcd_clear();
  lcd_xy(4, 1);
  lcd_puts("othello");
  flush_lcd();
}
/*************************************************************************************/


/********************************** マトリックスLED ************************************/
//指定した列の赤緑データをマトリックスLEDに出力
void col_out(int col, unsigned int rg_data)
{
	int i;

    for(i = 0; i < MAT_WIDTH * 2; i++)
	{
		if(rg_data & (1 << i))
		{
			SERIAL_SINK;          //点灯(カソード側吸い込み)
		}
		else
		{
			SERIAL_SOURCE;        //消灯(カソード側吐き出し)
		}

		SEND_LATCH_CLK;         //ラッチ
	}

	COL_EN = 0;               //全消灯

	LATCH_OUT;                //ラッチ出力

	COL_EN = 1 << col;         //点灯列指定
}
/******************************************************************************************/


/*********************************** ロータリーエンコーダ ***********************************/
//位相計数用レジスタのカウント数を0初期化
void clear_pulse_diff_cnt(void)
{
    MTU1.TCNT = 0;
}

//構造体初期化
void init_Rotary(struct Rotary *r)
{
    r->current_cnt = 0;
    r->prev_cnt    = 0;
}

//位相計数用レジスタからカウント数を読み取る
unsigned int read_rotary(void)
{
    return MTU1.TCNT;
}

//アンダーフロー判定
//位相計数を PULSE_DIFF_PER_CLICK で割って 0～65535/4 にスケーリング
int is_underflow(struct Rotary *r)
{
    return (UINT16T_MAX/PULSE_DIFF_PER_CLICK == r->current_cnt) && (r->prev_cnt == 0);
}

//オーバフロー判定
//位相計数を PULSE_DIFF_PER_CLICK で割って 0～65535/4 にスケーリング
int is_overflow(struct Rotary *r)
{
    return (UINT16T_MAX/PULSE_DIFF_PER_CLICK == r->prev_cnt) && (r->current_cnt == 0);
}

//左回りしたか
int is_rotary_turned_left(struct Rotary *r)
{           //カウンター増加　　　　　　　　または     オーバーフロー
    return ((r->current_cnt > r->prev_cnt) || is_overflow(r));
}

//右回りしたか
int is_rotary_turned_right(struct Rotary *r)
{           //カウンター減少               または     アンダーフロー
    return ((r->current_cnt < r->prev_cnt) || is_underflow(r));
}
/**************************************************************************************/


/************************************* オセロ ********************************************/
/************************************** コマ ********************************************* */
//Stone初期化
void init_Stone(void)
{
    int i;

    //置きコマ全撤去
    for(i = 0; i < MAT_WIDTH; i++)
    {
        red.stone[i] = 0x00;
        green.stone[i] = 0x00;
    }

    red.can_place   = 1;
    green.can_place = 1;
}

//指定した色のインスタンスを取得
volatile struct Stone *get_Stone_instance(enum stone_color sc)
{
	return (sc == stone_red) ? &red : &green;
}

//何も置かれてないか, または何色が置かれているか
enum stone_color read_stone_at(int x, int y)
{
    if(red.stone[x] & (1 << y))
    {
        return stone_red;
    }
    else if(green.stone[x] & (1 << y))
    {
        return stone_green;
    }
    else
    {
        return stone_black;
    }
}

//指定した色のコマを置く
void place(int x, int y, enum stone_color sc)
{
    get_Stone_instance(sc)->stone[x] |= (1 << y);
}

//指定した座標のコマを消す
void delete(int x, int y)
{   //赤緑どちらも消す
    red.stone[x]   &= ~(1 << y); 
    green.stone[x] &= ~(1 << y);
}

//盤面初期化
void init_board(void)
{
    //初期置きコマ
    place(3, 3, stone_red);
    place(4, 4, stone_red);
    place(3, 4, stone_green);
    place(4, 3, stone_green);
}
/*****************************************************************************/


/****************************** カーソル **************************************/
//カーソルの座標をセット
void set_cursor_xy(int x, int y)
{
    cursor.x = x;
    cursor.y = y;
}

void init_Cursor(void)
{
    //カーソルの色
    cursor.color = stone_red;

    //カーソルの初期座標
    set_cursor_xy(5, 3);
}

//上下左右を指定してカーソルの座標を更新
void move_cursor(int direction)
{
    int cx = cursor.x;
    int cy = cursor.y;

    if (direction == LEFT)  //左
    {
        cx--; //左シフト

        if (cx < 0) //左端まで行ったら一つ上の行の右端にワープ
        {
        	cx = MAT_WIDTH - 1;
        	cy++;

            if (cy > MAT_HEIGHT - 1) //上端を超えたら一番下の行にワープ
            {
            	cy = 0;
            }
        }
    }
    else if (direction == RIGHT)  // 右
    {
    	cx++; //右シフト

        if (cx > MAT_WIDTH - 1) //右端まで行ったら一つ下の行の左端にワープ
        {
        	cx = 0;
        	cy--;

            if (cy < 0) //下端を超えたら一番上の行にワープ
            {
            	cy = MAT_HEIGHT - 1;
            }
        }
    }
    else if (direction == UP) //上
    {
    	cy++; //上移動

		if (cy > MAT_HEIGHT - 1) //上端まで行ったら一つの右の列の下端にワープ
		{
			cx++;
			cy = 0;

			if (cx > MAT_WIDTH - 1) //右端を超えたら一番左の列にワープ
			{
				cx = 0;
			}
		}
    }
    else if(direction == DOWN) //下
    {
    	cy--; //下移動

		if (cy < 0) //下端まで行ったら一つ左の行の上端にワープ
		{
			cy = MAT_HEIGHT - 1;
			cx--;

			if (cx < 0) //左端を超えたら一番右の列にワープ
			{
				cx = MAT_WIDTH - 1;
			}
		}
    }

    set_cursor_xy(cx, cy);
}
/**********************************************************************************/


/************************************ ゲームロジック *********************************/
//座標範囲外か
int is_out_of_board(int x, int y)
{
    return ((x < 0) || (y < 0) || ( x > MAT_WIDTH  - 1) || (y > MAT_HEIGHT - 1));
}

//8方向のひっくり返しフラグを作る
//　      右下 右上  左下 左上  右   左  下   上
//flag :  b7   b6   b5   b4   b3   b2   b1  b0
//bit  :  0..その方角にひっくり返せない, 1..その方角にひっくり返せる
unsigned char make_flip_dir_flag(int x, int y, enum stone_color sc)
{
    int i, j;
    int dx, dy;
    unsigned char flag = 0x00;

    enum stone_color search;

    for(i = 0; i < 8; i++)
    {
        dx = dy = 0;

        for(j = 0; j < 8; j++)
        {
            dx += dxdy[i][0];                         
            dy += dxdy[i][1];

            if(is_out_of_board(x + dx, y + dy)) break; //範囲外ならbreak
    
            search = read_stone_at(x + dx, y + dy);    //コマの色を調査

            if(search == stone_black) break;           //何も置かれていなかったらbreak

            if(search == sc)                           //挟む側のコマの色に遭遇したら
            {
                if(j > 0)                              //j > 0 の時点で相手色を少なくとも1つは挟んでいる
                {
                    flag |= (1 << i);
                }

                break;
            }
        }
    }

    return flag;
}

//その場所にその色は置けるか？
int is_placeable(int x, int y, enum stone_color sc)
{
    unsigned char flag;

    if(read_stone_at(x, y) != stone_black) return 0;  //何かおいてあったらだめ

    flag = make_flip_dir_flag(x, y, sc);              //8方向フラグ作成

    return (flag != 0x00);                            //flag != 0x00なら少なくとも1方向は挟める
}

//8方向フラグをつかって相手のコマをひっくり返す
//置きチェックの責任はis_placeable関数にあるので一緒に使う
void flip_stones(unsigned char flag, int x, int y, enum stone_color sc)
{
    int i, j;
    int dx, dy;
    enum stone_color search;

    for(i = 0; i < 8; i++)
    {
        dx = dy = 0;

        if(flag & (1 << i))
        {
            for(j = 0; j < 8; j++)
            {
                dx += dxdy[i][0];
                dy += dxdy[i][1];

                search = read_stone_at(x + dx, y + dy); //コマの色をチェック

                if(search == sc)                        //置きチェック済みなので確認するのは自分の色が出たかのみ
                {
                    break;
                }

                delete(x + dx, y + dy); //コマを消す

                place(x + dx, y + dy, (search == stone_red) ? stone_green : stone_red)); //新しくコマを置く
            }
        }
    }
}

//ボード上にその色のコマが置ける場所は残っているか
int search_placeable(enum stone_color sc)
{
    int x, y;

    for(x = 0; x < MAT_WIDTH; x++)
    {
        for(y = 0; y < MAT_HEIGHT; y++)
        {
            if(is_placeable(x, y, sc)) 
            {
                return 1;
            }
        }
    }

    return 0;
}

//どっちも置けない場合は終わり
int is_game_over(enum stone_color sc1, enum stone_color sc2)
{
    return (!get_Stone_instance(sc1)->can_place && !get_Stone_instance(sc2)->can_place);
}
/***********************************************************************/


/******************************** 割込み ********************************/
// CMT1 CMI1 1msタイマ割込み
void Excep_CMT1_CMI1(void)
{
	int cn; //column number

    tc_1ms++;

    cn = tc_1ms % 8;

    //0.1秒おきに移動中のコマを点滅表示
    if((tc_1ms / 100) % 2)  //点灯
    {   
        //赤緑のコマでスクリーンを上書き
    	screen[cn] = (red.stone[cn] << 8) | (green.stone[cn]);

        //カーソルを追加
        if(cursor.color == stone_red)
        {
            screen[cursor.x] |= 1 << (cursor.y + 8);
        }
        else
        {
            screen[cursor.x] |= 1 << cursor.y;
        }
    }
    else //消灯
    {   //カーソルの下に置きコマがあったら
    	if(  (cn == cursor.x)  && ((red.stone[cn] | green.stone[cn]) & (1 << cursor.y)))
    	{   //その座標の置きコマも点滅させる
    		screen[cn] = ((red.stone[cn] & ~(1 << cursor.y)) << 8) | ((green.stone[cn]) & ~(1 << cursor.y));
    	}
    	else
    	{   //それ以外の場合は移動中のコマのみ消灯
    		screen[cn] = (red.stone[cn] << 8) | (green.stone[cn]);
    	}
    }

    col_out(cn, screen[cn]);
}

// ICU IRQ1 SW7立下がり割込み
void Excep_ICU_IRQ1(void)
{
    sw7_flag = 1;
}
/**************************************************************************************************/
/******************************************* 関数定義終 ********************************************/


/******************************************** メイン ***********************************************/
void main(void)
{
    enum   State  state;                //状態管理
    struct Rotary rotary;               //ロータリーエンコーダー
    unsigned char flip_dir_flag = 0x00; //置きチェック8方向フラグ

    init_HARDWARE();      //ハードウェア初期化
  
    state = INIT;

    while(1)
    {
        switch(state)
        {
            case INIT:
                clear_pulse_diff_cnt(); //位相計数レジスタを0クリア
                init_Rotary(&rotary);   //ロータリーエンコーダー構造体初期化
                init_Stone();           //コマ構造体初期化
                init_Cursor();          //カーソル構造体初期化
                init_board();           //ボード配置初期化
                init_display();         //LCD表示初期化
                state = MOVE;
                break;
            case MOVE:
                if(!sw7_flag)
                {
                    rotary.current_cnt = read_rotary() / PULSE_DIFF_PER_CLICK; 

                    if(is_rotary_turned_left(&rotary))
                    {
                        move_cursor((MOVE_TYPE_UP_DOWN) ? DOWN : LEFT);
                    }

                    if(is_rotary_turned_right(&rotary))
                    {
                        move_cursor((MOVE_TYPE_UP_DOWN) ? UP   : RIGHT);
                    }

                    rotary.prev_cnt = rotary.current_cnt;                     
                }
                else                                                          
                {
                    sw7_flag = 0;

                    state = PLACE;
                }
                break;
            case PLACE:
                if(get_Stone_instance(cursor.color)->can_place)
                {
                    if(is_placeable(cursor.x, cursor.y, cursor.color))
                    {
                        place(cursor.x, cursor.y, cursor.color);

                        flip_dir_flag = make_flip_dir_flag(cursor.x, cursor.y, cursor.color);
                    
                        state = FLIP;
                    }
                    else
                    {
                        state = MOVE;
                    }
                }
                else
                {
                    state = TURN_OVER;
                }
                break;
            case FLIP:
                flip_stones(flip_dir_flag, cursor.x, cursor.y, cursor.color);
                state = TURN_OVER;
                break;
            case TURN_OVER:
                cursor.color = (cursor.color == stone_red) ? stone_green : stone_red);;

                get_Stone_instance(cursor.color)->can_place = search_placeable(cursor.color);
             
                if(is_game_over(stone_red, stone_green))
                {
                    state = GAME_OVER;
                }
                else
                {
                    state = MOVE;
                }
                break;
            case GAME_OVER: //未完成
                lcd_clear();
                lcd_puts("Game over");
                lcd_xy(1, 2);
                lcd_puts("Push SW7");
                flush_lcd();
                while(!sw7_flag);
                sw7_flag = 0;
                state = INIT;
                break;
            default:
                break;
        }
    }
}

#ifdef __cplusplus
extern "C" void abort(void) {}
#endif


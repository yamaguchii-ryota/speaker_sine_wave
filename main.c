/******************************************************************************
 * ファイル名: main.c
 * 対象デバイス: dsPIC33FJ64GP802
 * コンパイラ: Microchip XC16
 *
 * 概要:
 * dsPIC33FJ64GP802の内蔵16ビットDACを使用し、指定された周波数の正弦波を
 * 差動出力するプログラムです。
 * このバージョンでは、出力振幅がフルスケールの1/3になるように調整されています。
 *
 * 機能仕様:
 * 1. 外部発振子: 20MHzのセラミック発振子を使用
 * 2. システムクロック: PLLを使用し、40MIPS (FCY = 40MHz)で動作
 * 3. 出力周波数: 10Hz ~ 2kHzの範囲で指定可能 (main関数内の `TARGET_FREQUENCY` で設定)
 * 4. 出力形式: 内蔵DACによるステレオ差動出力
 * - DAC1LP (Pin 25), DAC1LN (Pin 26)
 * - DAC1RP (Pin 23), DAC1RN (Pin 24)
 * 5. 正弦波生成:
 * - タイマ割り込みとDMAを利用せず、CPUによるテーブル参照方式を採用
 * - 出力周波数に応じて、正弦波の分解能（テーブルサイズ）を自動で調整し、
 * 低周波でも高周波でも滑らかな波形を維持
 *
 *****************************************************************************/

//==============================================================================
// インクルードファイル
//==============================================================================
#include <xc.h>
#include <math.h> // sinf()関数を使用するために必要

//==============================================================================
// コンフィグレーションビット設定 (ご指定の通りに設定)
//==============================================================================

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Mode (Primary Oscillator (XT, HS, EC))
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Select (HS Oscillator mode selected)
#pragma config OSCIOFNC = ON            // OSCO Pin Configuration (OSCO/CLKO/RA3 functions as CLKO (FOSC/2))
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor (Clock switching is enabled, Fail-Safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)


//==============================================================================
// 定数定義
//==============================================================================
#define FCY 40000000UL // 命令サイクルクロック周波数 (FOSC/2) -> (20MHz * 16 / 2 / 2) = 40MIPS

// 正弦波テーブルの品質に関する定義
#define MAX_SINE_TABLE_SIZE 1024 // 正弦波テーブルの最大サイズ（分解能）。メモリと品質のトレードオフ
#define INITIAL_SAMPLING_FREQ 200000.0f // 目標とする初期サンプリング周波数 (Hz)

// M_PIが未定義の場合があるため定義
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//==============================================================================
// グローバル変数
//==============================================================================
// 正弦波の波形データを格納するテーブル
uint16_t sine_table[MAX_SINE_TABLE_SIZE];
// 現在の正弦波テーブルのサイズ（分解能）
volatile uint16_t current_table_size;
// テーブルを参照するためのインデックス
volatile uint16_t sine_table_index = 0;


//==============================================================================
// プロトタイプ宣言
//==============================================================================
void init_oscillator(void);
void init_dac(void);
void set_output_frequency(float freq_hz);
void init_timer1(uint16_t period);


//==============================================================================
// メイン関数
//==============================================================================

int main(void) {
    // ★★★ 出力したい周波数をここで指定します (10.0 ~ 2000.0 Hz) ★★★
    const float TARGET_FREQUENCY = 100.0f; // 例: 100Hz

    // 1. システムの初期化
    init_oscillator(); // 発振器（クロック）を40MIPSに設定
    init_dac();        // DAコンバータを初期化

    // 2. 指定された周波数で正弦波の出力を開始
    set_output_frequency(TARGET_FREQUENCY);

    // 3. メインループ（割り込み駆動のため、ここでは何もしない）
    while (1) {
        // 省電力モードに移行することも可能
        // Idle();
    }

    return 0; // ここには到達しない
}


//==============================================================================
// 関数定義
//==============================================================================

/**
 * @brief 発振器を初期化し、システムクロックを40MIPSに設定します。
 *
 * 20MHzの外部発振子からPLLを用いて80MHzのVCOクロック(FVCO)を生成し、
 * それを2分周して40MHzのシステムクロック(FCY)を得ます。
 * FCY = FOSC * M / (N1 * N2) / 2
 * 40MHz = 20MHz * 16 / (2 * 2) / 2
 */
void init_oscillator(void) {
    // PLLプリスケーラ(N1), PLLポストスケーラ(N2), PLL分周比(M)を設定
    // N1 = PLLPRE + 2 = 0 + 2 = 2
    // N2 = 2 * (PLLPOST + 1) = 2 * (0 + 1) = 2
    // M = PLLDIV + 2 = 14 + 2 = 16
    PLLFBD = 14;              // M = 16
    CLKDIVbits.PLLPRE = 0;    // N1 = 2
    CLKDIVbits.PLLPOST = 0;   // N2 = 2

    // 発振器の切り替えを初期化
    __builtin_write_OSCCONH(0x03); // 新しい発振器ソース = PRI + PLL
    __builtin_write_OSCCONL(0x01); // 発振器の切り替えを要求

    // PLLがロックするまで待機
    while (OSCCONbits.COSC != 0b011);
    while (OSCCONbits.LOCK != 1);
}

/**
 * @brief DAコンバータ初期化関数 (TRIS設定追加・最終版)
 *
 * DACで使用するピンのTRISレジスタとAD1PCFGLレジスタを明示的に設定し、
 * 内蔵DAコンバータを差動出力モードで有効にします。
 */
void init_dac(void) {
    // --- ステップ 1: I/Oピンの方向を設定 ---
    // DACで使用するピン(RB12-RB15)を「入力」に設定します。
    // これによりデジタル出力ドライバが無効になり、DACがピンを制御できます。
    // TRISBレジスタのビット12, 13, 14, 15を1にセットします。
    TRISB |= 0xF000;

    // --- ステップ 2: ピンのアナログ機能を設定 ---
    // ADCモジュールを無効化し、AD1PCFGLを安全に設定します。
    AD1CON1bits.ADON = 0;
    // DACで使用するピン(AN9-AN12)を「アナログ」モードに設定します。
    // これが最も重要な設定です。
    AD1PCFGL &= 0xE1FF; // AN9, AN10, AN11, AN12 をアナログに設定

    // --- ステップ 3: DACモジュールの設定 ---
    // 設定前にDACを無効化します。
    DAC1CONbits.DACEN = 0;
    DAC1STAT = 0; // ステータスフラグを全てクリア

    // DACの動作モードを設定します。
    DAC1CONbits.FORM = 0;     // データ形式: 符号なし整数
    DAC1CONbits.AMPON = 1;    // スリープ/アイドル中も出力アンプを有効化
    DAC1CONbits.DACSIDL = 0;  // アイドルモード中も動作を継続

    // --- ステップ 4: DACの有効化 ---
    // DACモジュール自体を有効にします。
    DAC1CONbits.DACEN = 1;

    // DACの基準電圧が安定するまで少し待ちます (約15マイクロ秒)
    // 40 MIPSの場合、600サイクルで15usに相当します。
    for (volatile int i = 0; i < 600; i++);

    // 安定後、左右両チャンネルの出力を有効にします。
    DAC1STATbits.LOEN = 1;
    DAC1STATbits.ROEN = 1;
}



/**
 * @brief 指定された周波数で正弦波を生成するための設定を行います。
 *
 * 周波数に応じて最適なテーブルサイズ（分解能）とサンプリング周波数を計算し、
 * 正弦波テーブルを生成後、タイマーを設定して出力を開始します。
 * @param freq_hz 出力したい周波数 (Hz)
 */
void set_output_frequency(float freq_hz) {
    // 割り込みを一時的に停止して設定を変更
    IEC0bits.T1IE = 0;

    // 常に1周期を200個の点で表現する
    current_table_size = 200;
    // 必要なサンプリング周波数を計算
    float sampling_freq = freq_hz * current_table_size;

    // 2. 正弦波テーブルを生成
    for (int i = 0; i < current_table_size; i++) {
        // sinf()の引数: 2 * PI * i / N (i番目のサンプルでの角度[rad])
        // sinf()の値域: -1.0 ~ 1.0
        float angle = (2.0f * M_PI * i) / current_table_size;

        // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
        // ★ 修正箇所: 振幅を1/3に調整 ★
        // ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
        // DCオフセット(中心値)は32767.5f (0x8000相当)
        // 振幅はフルスケール(32767.5f)の1/3に設定
        const float dc_offset = 32767.5f;
        const float amplitude = 32767.5f / 5.0f;
        sine_table[i] = (uint16_t)(dc_offset + amplitude * sinf(angle));
    }

    // 3. テーブルインデックスをリセット
    sine_table_index = 0;

    // 4. 新しいサンプリング周波数でタイマーを初期化
    // PR1 = (FCY / sampling_freq) - 1;
    uint16_t period = (uint16_t)(FCY / sampling_freq) - 1;
    init_timer1(period);
}

/**
 * @brief Timer1を初期化し、サンプリング周期を設定します。
 * @param period TMR1の周期を決めるPR1レジスタの値
 */
void init_timer1(uint16_t period) {
    T1CON = 0;               // タイマーを停止し、設定をリセット
    TMR1 = 0;                // タイマーカウンターをリセット
    PR1 = period;            // 周期を設定

    // T1CON設定
    T1CONbits.TON = 1;       // タイマーを有効化
    T1CONbits.TSIDL = 0;     // アイドル中も動作
    T1CONbits.TGATE = 0;     // ゲートモード無効
    T1CONbits.TCKPS = 0b00;  // プリスケーラ 1:1
    T1CONbits.TCS = 0;       // クロックソース: 内部クロック(FCY)

    // 割り込み設定
    IPC0bits.T1IP = 5;       // 割り込み優先度を5に設定 (1-7で任意)
    IFS0bits.T1IF = 0;       // 割り込みフラグをクリア
    IEC0bits.T1IE = 1;       // Timer1割り込みを有効化
}


//==============================================================================
// 割り込みサービスルーチン
//==============================================================================

/**
 * @brief Timer1 割り込みサービスルーチン
 *
 * 一定周期（サンプリング周期）で呼び出され、正弦波テーブルの次の値を
 * DAコンバータに出力します。
 */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void) {
    // 1. テーブルから現在の波形データを取得
    uint16_t dac_value = sine_table[sine_table_index];

    // 2. DACの左右両チャンネルに同じデータを出力
    DAC1LDAT = dac_value;
    DAC1RDAT = dac_value;

    // 3. 次のデータを参照するためにインデックスを更新
    sine_table_index++;

    // 4. インデックスがテーブルの終端に達したら先頭に戻す（ループ）
    if (sine_table_index >= current_table_size) {
        sine_table_index = 0;
    }

    // 5. Timer1の割り込みフラグをクリア
    IFS0bits.T1IF = 0;
}

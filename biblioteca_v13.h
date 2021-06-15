#ifndef BIBLIOTECA_V13_H
#define BIBLIOTECA_V13_H

#pragma warning disable 520,1498

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 20000000                                                     // Determina cristal atual, uso obrigatorio para "delay".
#endif

///* -   Prototipos   - */
//#ifdef motorPasso
//void motorPassoIniciar(void);
//void motorPasso(unsigned char mp01Tipo,unsigned int mp01Passos,unsigned char mp01Sentido);
//#endif
//#ifdef _UART==1
//void uartIniciarStd(void);
//unsigned char uartIniciarCfg(const long int baudrate);
//void uartTXC(const char dado);
//unsigned char uartRXC(void);
//void uartTXT(const char *texto);
//#endif
//#ifdef adc
//void adcIniciar(void);
//void adcIniciarCanal(unsigned char numCanal);					// Inicializa o hardware com numero de canal(ais).
//unsigned int adcLer(void);
//unsigned int adcCanal(unsigned char vlrCanal);					// Função de seleção e leitura do módulo.
//#endif
//void lcd4Bits(unsigned char valor8Bits, unsigned char valorRS);
//void lcdIniciar(volatile unsigned char *gate, unsigned char lcdTipo);
//void lcdIniciarXm118(void);
//unsigned char lcdBotao(void);
//void lcdPosicao(const char lcd01X, const char lcd01Y);
//void lcdCaracter(const char lcd01Char, const char linha, unsigned char coluna);
//void lcdTexto(const char *lcd01Texto, const char linha, unsigned char coluna);
//#ifdef teclado
//void tecladoIniciar(void);
//unsigned char tecladoLer(void);
//#endif
//#ifdef dht
//void dhtxxIniciar(volatile unsigned char *gate, unsigned char dhtBitVlr); // Inicia o hardware e tipo do DHT
//unsigned char dhtxxLer(void); // Faz leitura e conversao dos bits do DHT.
//unsigned char dhtxxChecar(void); // Verifica pinos d DHT.
//unsigned char hexDec(unsigned char valor); // Converte valore (DHT11).
//unsigned char dhtxxValor(unsigned char valordht); // Faz leitura dos do DHT escolhido.
//#endif
#define bit1(valor,bit) valor |= (1<<bit)		// Faz o 'bit' de 'valor' =1
#define bit0(valor,bit) valor &= ~(1<<bit)		// Faz o 'bit' de 'valor' =0
#define bitI(valor,bit) valor ^= (1<<bit)		// Inverte o 'bit' de 'valor' 
#define bitX(valor,bit) (valor&(1<<bit))		// Testa e retorna o 'bit' de 'valor' 
#define testarBit(var,bit)	(unsigned)((var^bit)>>bit) & 0x01               // Testa e retorna valor do bit.

/* ------- LCD ------ */
static unsigned char *lcdPort;
unsigned const char lcdL1=0x80, lcdL2=0xC0;
unsigned char lcdType=0;
unsigned char lcdSerFlag=0;

//#ifdef	lcd2x16
//lcdType=0;
//#elif	lcd16x2
//lcdType=0;
//#endif
//
//#ifdef	lcd4x16
//lcdType=1;
//#elif	lcd16x4
//lcdType=1;
//#endif
//
//#ifdef	lcd1x20
//lcdType=2;
//#elif	lcd20x1
//lcdType=2;
//#endif
//
//#ifdef	lcd2x20
//lcdType=3;
//#elif	lcd20x2
//lcdType=3;
//#endif
//
//#ifdef	lcd4x20
//lcdType=4;
//#elif	lcd20x4
//lcdType=4;
//#endif
//
//#ifdef	lcd2x40
//lcdType=5;
//#elif	lcd40x2
//lcdType=5;
//#endif

#define lcd2x16	0
#define lcd16x2	0

#define lcd4x16	1
#define lcd16x4	1

#define lcd1x20	2
#define lcd20x1	2

#define lcd2x20	3
#define lcd20x2	3

#define lcd4x20	4
#define lcd20x4	4

#define lcd2x40	5
#define lcd40x2	5

/* ---- LCD XM118 ---- */
#define saidaLCD	PORTD			// Alias do PORT de saida.
#define confgLCD	TRISD			// Alias de configuracao do PORT.
#define saidaCtl	PORTE			// Saida do Controle LCD.
#define confgCtl	TRISE			// Config. o Controle LCD.
#define pinoLcdRs	PORTEbits.RE0	// Alias do pino 'RS'.
#define pinoLcdEn	PORTEbits.RE1	// Alias do pino 'E'.

/* ---- LCD Serial ---- */
//#define cfgSaidaLCD TRISE

#ifndef _PIC18									// Se for para 16F...
	#define lcd01PinoCk	    PORTEbits.RE0				// Pino de saída do 'clock'.
	#define lcd01PinoDt	    PORTEbits.RE1				// Pino de saída do 'data'.
	#define lcd01PinoEn	    PORTEbits.RE2				// Pino de saída do 'enable'.
#elif   _PIC18									// Se for para 18F...
	#define lcd01PinoCk	    LATEbits.LATE0				// Pino de saída do 'clock'.
	#define lcd01PinoDt	    LATEbits.LATE1				// Pino de saída do 'data'.
	#define lcd01PinoEn	    LATEbits.LATE2				// Pino de saída do 'enable'.
#endif

/* ---- Teclado ----- */
#define saidaTec	PORTB		// ...define o termo correto para a saida.
#define confgTec	TRISB		// Configura as entradas(1) e saidas(0).

/* - Motor de passo - */
#define pino1CFG    TRISDbits.TRISD0
#define pino2CFG    TRISDbits.TRISD1
#define pino3CFG    TRISDbits.TRISD2
#define pino4CFG    TRISDbits.TRISD3

#define pino1       PORTDbits.RD0
#define pino2       PORTDbits.RD1
#define pino3       PORTDbits.RD2
#define pino4       PORTDbits.RD3

#define mp01Tempo	8							// Constante de tempo para a sub rotina de motorPassoPausa.

/* ---- DHT 11/22 ---- */
static unsigned char *dhtPort;
static unsigned char dhtBit;


/* ---- Dec 2 ASCII---- */
void valorASCII(unsigned int valor, unsigned char *buffer,unsigned char digi) // Converte INT em ASCII: Valor(Bin), Matriz, Numero de digitos (0 a 5). 
{
	if(digi>5) digi=5;
	switch(digi)
	{
			case 0: 
				break;
			case 1:
				buffer[0]=(valor%10)       +0x30; // unidade
				break;
			case 2:
				buffer[0]=(valor/10)       +0x30; // dezena
				buffer[1]=(valor%10)       +0x30; // unidade
				break;
			case 3:
				buffer[0]=(valor/100)      +0x30; // centena
				buffer[1]=((valor/10)%10)  +0x30; // dezena
				buffer[2]=(valor%10)       +0x30; // unidade
				break;	
			case 4:
				buffer[0]=(valor/1000)     +0x30; // unidade de milhar
				buffer[1]=((valor/100)%10) +0x30; // centena
				buffer[2]=((valor/10)%10)  +0x30; // dezena
				buffer[3]=(valor%10)       +0x30; // unidade
				break;
			case 5:
				buffer[0]=(valor/10000)    +0x30; // dezena de milhar
				buffer[1]=((valor/1000)%10)+0x30; // unidade de milhar
				buffer[2]=((valor/100)%10) +0x30; // centena
				buffer[3]=((valor/10)%10)  +0x30; // dezena
				buffer[4]=(valor%10)       +0x30; // unidade
				break;
	}
}

/* -- Motor de passo -- */
#ifdef _MPASSO_

#ifndef _PIC18									// Se for 16F...
	#define mp01MAP	    pino1					// Pino saida da Bobina A+
	#define mp01MAN	    pino2					// Pino saida da Bobina A-
	#define mp01MBP	    pino3					// Pino saida da Bobina B+
	#define mp01MBN	    pino4					// Pino saida da Bobina B-

#elif   _PIC18									// Se for 18F...
	#define mp01MAP	    LATAbits.LATA1					// Pino saida da Bobina A+
	#define mp01MAN	    LATAbits.LATA2					// Pino saida da Bobina A-
	#define mp01MBP	    LATAbits.LATA3					// Pino saida da Bobina B+
	#define mp01MBN	    LATAbits.LATA5					// Pino saida da Bobina B-
#endif

unsigned char mp01Tab01[]={0X08,0X0C,0X04,0X06,0X02,0X03,0X01,0X09};            // Matriz mp01Tipo char, com valor dos mp01Passos (+). 
unsigned char mp01Tab02[]={0X09,0X01,0X03,0X02,0X06,0X04,0X0C,0X08};            // Matriz mp01Tipo char, com valor dos mp01Passos (-). 

void motorPassoIniciar(void)	// Configura o PORT para saida do motor de passo.
{
	pino1CFG=0;
	pino2CFG=0;
	pino3CFG=0;
	pino4CFG=0;
    mp01MAP=0;									// Desliga o pino para nao aquecer o motor.
    mp01MAN=0;									// Desliga o pino para nao aquecer o motor.
    mp01MBP=0;									// Desliga o pino para nao aquecer o motor.
    mp01MBN=0;									// Desliga o pino para nao aquecer o motor.
}

void motorPasso(unsigned char mp01Tipo,unsigned int mp01Passos,unsigned char mp01Sentido)
{
    unsigned char mp01Salto,mp01Pos,mp01Qtd,mp01X;                        // Variavel para uso no laco interno.
    unsigned char mp01PosMatriz;                         // Variavel para uso no laco emp01Xterno.
    unsigned char mp01TmpVlr01;							// Variavel para calculo, temporario.
    
	unsigned int numVezesEmp01XecTabela,mp01Y;
	
    switch (mp01Tipo)                                                           // Seleciona 'mp01Tipo'...
    {
        case 0:                                                                 // ...se 0(Passo Completo 1 bit)...
        {
            mp01Salto=0x02;                                                     // ...pula de 2 em 2 na tabela e...
            mp01Pos=0x00;                                                       // ...nao faz offset do inicio e...
            mp01Qtd=0x04;                                                       // ...ajusta a quantidade de mp01Passos.
            break;                                                              // Interrompe o processo.
        }
        case 1:                                                                 // ...se 1(Passo Completo 2 bits)...
        {
            mp01Salto=0x02;                                                     // ...pula de 2 em 2 na tabela e...
            mp01Pos=0x01;                                                       // ...faz offset do inicio e...
            mp01Qtd=0x04;                                                       // ...ajusta a quantidade de mp01Passos.
            break;                                                              // Interrompe o processo.
        }
        case 2:                                                                 // ...se 2(Meio Passo)...
        {
            mp01Salto=0x01;                                                     // ...pula de 1 em 1 na tabela e...
            mp01Pos=0x00;                                                       // ...nao faz offset do inicio e...
            mp01Qtd=0x08;                                                       // ...ajusta a quantidade de mp01Passos.
            break;                                                              // Interrompe o processo.
        }
    }
    
    numVezesEmp01XecTabela = (unsigned)(mp01Passos/mp01Qtd);			// Calcula numero de vezes de repeticao.
	    
    for(mp01Y=0;mp01Y<numVezesEmp01XecTabela;mp01Y++)                           // Laco para repetir a tabela.
    {
	mp01TmpVlr01=(unsigned)(0+mp01Pos);
        for(mp01X=mp01TmpVlr01;mp01X<8;mp01X+=mp01Salto)                        // Laco para mp01Posicao na tabela.
        {
            mp01PosMatriz=mp01X;                                                // Copia o valor de mp01Posicao.

            if (mp01Sentido==0)                                                 // Se for mp01Sentido horario...
            {
                mp01MAP=testarBit(mp01Tab01[mp01PosMatriz],0);			// Faz o pino de saida igual ao bit do numero.
                mp01MAN=testarBit(mp01Tab01[mp01PosMatriz],1);			// Faz o pino de saida igual ao bit do numero.
                mp01MBP=testarBit(mp01Tab01[mp01PosMatriz],2);			// Faz o pino de saida igual ao bit do numero.
                mp01MBN=testarBit(mp01Tab01[mp01PosMatriz],3);			// Faz o pino de saida igual ao bit do numero.

//                motorPassoPausa();						// Aguarda...
				__delay_ms(mp01Tempo);							// Atraso em ms(mili segundos) conforme variavel.
            }
            else                                                                // ...senao e mp01Sentido antihorario...
            {
                mp01MAP=testarBit(mp01Tab02[mp01PosMatriz],0);			// Faz o pino de saida igual ao bit do numero.
                mp01MAN=testarBit(mp01Tab02[mp01PosMatriz],1);			// Faz o pino de saida igual ao bit do numero.
                mp01MBP=testarBit(mp01Tab02[mp01PosMatriz],2);			// Faz o pino de saida igual ao bit do numero.
                mp01MBN=testarBit(mp01Tab02[mp01PosMatriz],3);			// Faz o pino de saida igual ao bit do numero.

//                motorPassoPausa();						// Aguarda...
				__delay_ms(mp01Tempo);							// Atraso em ms(mili segundos) conforme variavel.
            }
        }
    }
    mp01MAP=0;									// Desliga o pino para nao aquecer o motor.
    mp01MAN=0;									// Desliga o pino para nao aquecer o motor.
    mp01MBP=0;									// Desliga o pino para nao aquecer o motor.
    mp01MBN=0;									// Desliga o pino para nao aquecer o motor.
}

#endif

/* Comunicacao Serial */
#ifdef _UART_

void uartIniciarStd(void)					// Configura a velocidade da USART: 9600bps@20MHz.
{   
#ifdef _16F877A
    CMCON=0x07;
#endif
    SPBRG=31;				// Valor calculado p/9600bps@20MHz.
    TXSTAbits.SYNC=0;		// Modo assincrono.
    RCSTAbits.SPEN=1;		// Habilita comunicacao serial.
    TRISCbits.TRISC6=1;		// Config. pino TX (Datasheet pag.243)
    TRISCbits.TRISC7=1;		// Config. pino RX.
    RCSTAbits.CREN=1;		// Habilita recepcao continua.
    TXSTAbits.TXEN=1;		// Habilita transmissao.
}

unsigned char uartIniciarCfg(const long int baudrate)	// Configura a velocidade da USART: Retorna '1' como opcao valida.
{
    unsigned int x;								// Variável local temporária.
    x = (_XTAL_FREQ - baudrate*64)/(baudrate*64);				// Calcula o valor para o registro de controle...
    BRGH = 0;									// Bit(TXSTA)=0, gerador de baud em baixa velocidade.
  
    if(x>255)									// Se o calculo for maior que 255...
    {
		x = (_XTAL_FREQ - baudrate*16)/(baudrate*16);				// ...calcula o valor para o registro de controle e...
		BRGH = 1;								// ...faz Bit(TXSTA)=1, gerador de baud em alta velocidade.
    }
    if(x<256)									// Se o calculo final for menor que 256...
    {
		SPBRG = x;								// ...grava o valor calculado no registro SPBRG.
		SYNC = 0;								// ...bit(TXSTA)=0, modo assincrono...
		SPEN = 1;								// ...bit(RCSTA)=1, habilita comunicação serial...
		TRISCbits.TRISC6=1;							// ...configura o pino TX(Ver datasheet, item 20 pág.243)...
		TRISCbits.TRISC7=1;							// ...configura o pino RX...
		CREN = 1;								// ...bit(RCSTA)=1, habilita recepção contínua...
		TXEN = 1;								// ...bit(TXSTA)=1, habilita transmissão...
		return 1;								// ...retorna '1', se foi tudo bem...
    }
    return 0;									// ...retorna '0', quando calculo errado (Rever o valor de baud e XTAL).
}

void uartTXC(const char dado)			// Transmite um caracter pela serial.
{
    TXREG = dado;			// Quando TRMT=1, enviar 'dado'.
    while(!TXSTAbits.TRMT);	// Trava neste ponto ate liberar TXREG.
}

unsigned char uartRXC(void)				// Recebe um caracter pela serial.
{
	unsigned char tmrOut=0, kTemp=0;
	unsigned char rxValor=0;
    if(PIR1bits.RCIF)
	{
		while(!PIR1bits.RCIF)	// Trava neste ponto ate receber um dado.
		{
			kTemp++;
			if (kTemp>100) break;
		}
		rxValor = RCREG;
	}
    
    return (rxValor);			// Retorna com o dado recebido.
}

void uartTXT(const char *texto)			// Envia um texto(String) pela serial.
{
    unsigned char i;			// Variavel local temporaria.
    for(i=0;texto[i]!='\0';i++)	// De 0 ate o caracter diferente de nulo...
    uartTXC(texto[i]);			// ...transmitir caracteres.
}

#endif

/*        ADC         */
#ifdef _ADC_

void adcIniciar(void)					// Inicializa o ADC, somente canal AN0.
{
    TRISA=0x01;			// Somente AN0 como entrada.

#ifndef _PIC18			// Se nao for 18F...
    PORTA=0x00;			// Limpa PORT.
#elif   _PIC18			// Se for 18F...
    LATA=0x00;			// Limpa PORT.
#endif

    PIR1bits.ADIF = 0;	// Limpa Flag da interrupcao do modulo.
    PIE1bits.ADIE = 0;	// Desliga a interrupcao do ADC.

#ifndef _PIC18			// Se nao for 18F...
    ADCON0 = 0x80;      // Configura  ADC com Fosc/32 e CH0.
#endif

#ifdef _16F877A			// Se for o PIC16F877A...
    ADCON1 = 0x8E;      // Saida Justificada a direita e Vref(0 e 5V).
#endif

#ifdef _16F887			// Se for o PIC16F887...
    ADCON1 = 0x80;      // Saida Justificada a direita e Vref(0 e 5V).
    ANSEL = 0x01;		// Desliga analogicos de AN1 ate AN7.
#endif

#ifdef _18F4550			// Se for o PIC18F4550...
    ADCON1 = 0x0E;		// Seleciona Vref(0 e 5V) e AN0.
    ADCON2 = 0xBA;		// Just Right, 20TAD, FOSC/32
#endif
    __delay_ms(35);		// Aguarda estabilizar...
}

void adcIniciarCanal(unsigned char numCanal)	// Inicializa o ADC com numero de canal(ais) (0 a 15). Opcao em '0' entradas de TRISA modo digital.
{
	/* Atencao:
	 * Se 'numCanal = 0' entradas de TRISA modo digital
	 * Não é possivel escolher somente: 2, 3, 4 canais ou 7 canais sequenciais.
	 */
	unsigned char adTmpLac=0;
	TRISA=0;
	
#ifdef _16F877A
	if(numCanal > 7)numCanal = 7;
		// canal em uso:     0    1    5    5    5    5    6    8    8
	unsigned char adcTab[]={0x86,0x8E,0x82,0x82,0x82,0x82,0x89,0x80,0x80};
	
	do
	{
		bit1(TRISA,adTmpLac);
		adTmpLac++;
	}while(adTmpLac<numCanal);
		
	ADCON1 = adcTab[numCanal];
	
	PORTA=0x00;			// Limpa PORT.
#endif
	
#ifdef _16F887
	if(numCanal > 13)numCanal = 13;
//	PORTA=0x00;			// Limpa PORT.
#endif
	
#ifdef _18F4550
	if(numCanal > 12)numCanal = 12;
//	LATA=0x00;			// Limpa PORT.
#endif
	
    PIR1bits.ADIF = 0;	// Limpa Flag da interrupcao do modulo.
    PIE1bits.ADIE = 0;	// Desliga a interrupcao do ADC.
	
    __delay_ms(35);		// Aguarda estabilizar...
}

unsigned int adcLer(void)	// Seleciona canal 0 e retorna com valor (INT) (0 a 1023).
{
    unsigned int valorADC;	// Variavel local temporaria.
    ADCON0bits.ADON = 1;	// Habilita modulo.
    __delay_ms(1);			// Aguarda estabilizar...
    ADCON0bits.GO = 1;		// Inicia conversao.
    while(ADCON0bits.GO);	// Aguarda terminar a conversao.
    valorADC = (unsigned int) (ADRESH<<8) | ADRESL; // Unindo as partes.
    return(valorADC);		// Retorna com o valor convertido.
}

unsigned int adcCanal(unsigned char vlrCanal)	// Seleciona canal (0 a 15) e retorna com valor (INT) (0 a 1023).
{
    unsigned int vlrTmp;								// Variável local temporária.
	
#ifdef _16F877A
	ADCON0 = (vlrCanal<<3) | 0x80;	// Pocisiona o valor do canal no registro.
#endif
	
#ifdef _16F887
	ADCON0 = (vlrCanal<<2) | ADCON0;	// Pocisiona o valor do canal no registro.
#endif
	
#ifdef _16F4550
	ADCON0 = (vlrCanal<<2) | ADCON0;	// Pocisiona o valor do canal no registro.
#endif
	
//    if(vlrCanal==0)ADCON0 = 0x80;					// Canal AN0 selecionado.
//    if(vlrCanal==1)ADCON0 = 0x84;					// Canal AN1 selecionado.
//    if(vlrCanal==2)ADCON0 = 0x88;					// Canal AN2 selecionado.
	
    ADCON0bits.ADON = 1;							// Habilita módulo.
    __delay_ms(1);									// Aguarda...
    ADCON0bits.GO = 1;								// Inicia conversão.
    while(ADCON0bits.GO);							// Aguarda terminar a conversão.
    vlrTmp = (ADRESH<<8) | ADRESL;						// Unindo as partes para compor o valor.
    return (vlrTmp);								// Retorna com o valor bruto.
}

unsigned int adcVolts(unsigned char canal) // Retorna valor (INT) em milivolts (0 a 4995).
{
	unsigned int vlrVolts;
	const float cteVolt = 4.8828125;
	vlrVolts = (int) adcCanal(canal) * cteVolt;
	return (vlrVolts);
}

#endif

/* ------ LCD ------- */
void lcdSerial(unsigned char lcd01Valor, unsigned char lcd01Rs)                  // Sub rotina de conversão 4bits em serial
{
//	#define lcd01PinoCk	    PORTEbits.RE0				// Pino de saída do 'clock'.
//	#define lcd01PinoDt	    PORTEbits.RE1				// Pino de saída do 'data'.
//	#define lcd01PinoEn	    PORTEbits.RE2				// Pino de saída do 'enable'.

    unsigned char lcd01Mascara, lcd01Flag, lcd01Tmp;                            // Variáveis locais tipo 'char'.
    lcd01PinoEn = 0;                                                            // Prepara pino de 'enable' (=0).
    lcd01PinoCk = 0;                                                            // Prepara pino de 'clock' (=0).
    lcd01PinoDt = lcd01Rs;                                                      // Faz pino de RS igual a variável (RS=1 dado RS=0 comando).
    lcd01PinoCk = 1;                                                            // Ativa pino de 'clock'.
    __delay_us(2);								// Aguarda lcd01Tmpo de estabilização.
    lcd01PinoCk = 0;                                                         // Retorna pino ao estado original.
    lcd01Mascara = 0x08;                                                        // Máscara para realizar o deslocamento.
    
    for (lcd01Tmp=0; lcd01Tmp<4; lcd01Tmp++)                                    // Laço para executar o deslocamento.
    {
		lcd01Flag =(unsigned) lcd01Valor & lcd01Mascara;			// Variável determina valor do bit no dado.
		if(lcd01Flag==0) lcd01PinoDt = 0;							// Verifica se o bit for 0...coloca o pino em 0.
		else lcd01PinoDt = 1;								// ...se não coloca o pino em 1.
		lcd01PinoCk = 1;							// Ativa pino de 'clock'. 
		__delay_us(2);								// Aguarda lcd01Tmpo de estabilização.
		lcd01PinoCk = 0;							// Retorna pino ao estado original.
		lcd01Mascara =(unsigned) lcd01Mascara >> 1;				// Desloca um bit na máscara.
    }
  
    lcd01PinoCk = 1;                                                            // Ativa pino de 'clock'. Mais um clock, pois SC e ST estão juntos.
    __delay_us(2);								// Aguarda lcd01Tmpo de estabilização.
    lcd01PinoCk = 0;                                                            // Retorna pino ao estado original.
    lcd01PinoDt = 0;                                                            // Desativa o pino.
    lcd01PinoEn = 1;                                                            // Ativa o pino de 'enable'.
    __delay_us(2);								// Aguarda lcd01Tmpo de estabilização.
    lcd01PinoEn = 0;                                                            // Desativa o pino de 'enable'.
}

void lcd4Bits(unsigned char valor8Bits, unsigned char valorRS)			// Converte 8bits em 2x 4bits: Valor em 8Bits, Valor RS('0'=comando).
{
	extern unsigned char *lcdPort;				// Var externa

    *lcdPort = valor8Bits >> 4;					// Desloca MSB pra saida
	if(valorRS==0)	bit0(*lcdPort,4);			// RS=0(Comando)|RS=1(Dado)
	if(valorRS==1)	bit1(*lcdPort,4);			// RS=0(Comando)|RS=1(Dado)

	bit1(*lcdPort,5);							// Gera pulso no pino 'E'
	bit0(*lcdPort,5);							// 
	__delay_ms(5);								// Aguarda estabilizar.
	
    *lcdPort=valor8Bits;						// Coloca LSB na saida.

	if(valorRS==0)	bit0(*lcdPort,4);			// RS=0(Comando)|RS=1(Dado)
	if(valorRS==1)	bit1(*lcdPort,4);			// RS=0(Comando)|RS=1(Dado)

	bit1(*lcdPort,5);							// Gera pulso no pino 'E'
	bit0(*lcdPort,5);							// 
	__delay_ms(5);								// Aguarda estabilizar.
}

void lcdSerIniciar(unsigned char *lcdSGate, unsigned char lcdSTipo, unsigned char pinCk, unsigned char pinDt, unsigned char pinEn)
{
	unsigned char *cfgSerGate, t0Ser, cfgTris=0xFF;
	extern unsigned char *lcdPort;				// Var externa
	
	ADCON1 = 0x8E;      	// Desliga analogicos do PORTE.
	
	lcdType=lcdSTipo;	// Tipo do LCD: 16x2, 20x4...
		
	bit0(cfgTris,pinCk); // Configura como saida o pino de clock.
	bit0(cfgTris,pinDt); // Configura como saida o pino de data.
	bit0(cfgTris,pinEn); // Configura como saida o pino de enable.
	
	t0Ser = (char) lcdSGate; // Puxa endereco fisico (PORT)
	lcdPort = t0Ser; // Salva o PORT(int->char)
	
	cfgSerGate = (t0Ser + 0x80); // Carrega o TRISx
	*cfgSerGate=cfgTris;		 // Configura TRISx
	*lcdPort = 0x00;							// Limpa o Port
	
//	#define lcd01PinoCk	    PORTEbits.RE0				// Pino de saída do 'clock'.
//	#define lcd01PinoDt	    PORTEbits.RE1				// Pino de saída do 'data'.
//	#define lcd01PinoEn	    PORTEbits.RE2				// Pino de saída do 'enable'.
	
//	lcdSerial(0x02,0);							// Cursor em Home.
//	lcdSerial(0x28,0);							// Bus de 4bits e 2 linhas.
//	lcdSerial(0x0E,0);							// Liga display e cursor.
////	lcdSerial(0x0C,0);							// Liga display.
//	lcdSerial(0x01,0);							// Limpa o LCD.
	
	lcdSerial(0x37,0);							// Cursor em Home.
	lcdSerial(0x28,0);							// Bus de 4bits e 2 linhas.
	
}

void lcdIniciar(volatile unsigned char *gate, unsigned char lcdTipo)	// Inicializa o LCD da PCI com botoes: &PORTx, Tipo(16x2, 20x4...).
{
	unsigned char *cfgGate, t0;
	extern unsigned char *lcdPort;				// Var externa
	
	lcdType=lcdTipo;	// Tipo do LCD: 16x2, 20x4...
//	lcdSerFlag=lcdPin;			// LCD pinagem: '0'=Paralelo, '1'=Serial
	
	t0 = (char) gate;									// Puxa endereco fisico (PORT)
	lcdPort = t0;								// Salva o PORT(int->char)
	
	cfgGate = (t0 + 0x80);						// Carrega o TRISx
	*cfgGate = 0xC0;							// Configura TRISx
	*lcdPort = 0x00;							// Limpa o Port

//	if(lcdPin==0)
//	{
		lcd4Bits(0x02,0);							// Cursor em Home.
		lcd4Bits(0x28,0);							// Bus de 4bits e 2 linhas.
//	    lcd4Bits(0x0E,0);							// Liga display e cursor.
		lcd4Bits(0x0C,0);							// Liga display.
		lcd4Bits(0x01,0);							// Limpa o LCD.
//	}
//	else
//	{
//		lcdSerial(0x02,0);							// Cursor em Home.
//		lcdSerial(0x28,0);							// Bus de 4bits e 2 linhas.
////	    lcdSerial(0x0E,0);							// Liga display e cursor.
//		lcdSerial(0x0C,0);							// Liga display.
//		lcdSerial(0x01,0);							// Limpa o LCD.
//	}
	
}

void lcdIniciarXm118(void)		// Inicializa o LCD do kit 8 bits(XM118).
{
	lcdSerFlag=0;				// LCD pinagem em paralelo.
	ADCON1 = 0x8E;      	// Desliga analogicos do PORTE.
    confgLCD=0x00;			// Configura entrada/saida dos pinos do PORT.
    saidaLCD=0x00;			// Limpa o PORT.
    confgCtl=0x00;			// Configura entrada/saida dos pinos do PORT.
    saidaCtl=0x00;			// Limpa o PORT.
    lcd4Bits(0x02,0);		// Cursor em Home.
    lcd4Bits(0x28,0);		// Bus de 4bits e 2 linhas.
    lcd4Bits(0x0C,0);		// Liga só o display.
//    lcd4Bits(0x0E,0);		// Liga display e cursor.
    lcd4Bits(0x01,0);		// Limpa o LCD.
}

unsigned char lcdBotao(void)	// Retorna o valor dos botoes: (3 a 0). 
{
	extern unsigned char *lcdPort;				// Var externa
	unsigned char result=0;
	if(bitX(*lcdPort,6)) bit1(result,0);
	if(bitX(*lcdPort,7)) bit1(result,1);
	return (result);
}

void lcdPosicao(const char lcd01X, const char lcd01Y)	// Posiciona o cursor: Linha, Coluna.
{
	unsigned char lcd01Tmp;				// Variavel local tipo 'char'.
	
	switch(lcdType)
	{
		case 0:
			if (lcd01X == 1) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 2) lcd01Tmp = (unsigned)192 + lcd01Y;	// A soma define posicao da coluna.
			break;
		case 1:
			if (lcd01X == 1) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 2) lcd01Tmp = (unsigned)191 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 3) lcd01Tmp = (unsigned)143 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 4) lcd01Tmp = (unsigned)207 + lcd01Y;	// A soma define posicao da coluna.
			break;
		case 2:
			if (lcd01X < 11) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X > 10) lcd01Tmp = (unsigned)192 + lcd01Y;	// A soma define posicao da coluna.
			break;
		case 3:
			if (lcd01X == 1) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 2) lcd01Tmp = (unsigned)192 + lcd01Y;	// A soma define posicao da coluna.
			break;
		case 4:
			if (lcd01X == 1) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 2) lcd01Tmp = (unsigned)191 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 3) lcd01Tmp = (unsigned)147 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 4) lcd01Tmp = (unsigned)211 + lcd01Y;	// A soma define posicao da coluna.
			break;
		case 5:
			if (lcd01X == 1) lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna.
			if (lcd01X == 2) lcd01Tmp = (unsigned)192 + lcd01Y;	// A soma define posicao da coluna.
	}
	
//    unsigned char lcd01Tmp;				// Variavel local tipo 'char'.
//    lcd01Tmp = (unsigned)127 + lcd01Y;	// A soma define posicao da coluna e...
//    if (lcd01X == 2) lcd01Tmp += lc2;	// ...verifica se deve somar para a 2 linha.
//    if (lcd01X == 3) lcd01Tmp += lc3;	// ...verifica se deve somar para a 3 linha.
//    if (lcd01X == 4) lcd01Tmp += lc4;	// ...verifica se deve somar para a 4 linha.
    lcd4Bits(lcd01Tmp,0);				// Chama sub rotina de comando (linha e coluna).
	
	
	
	
}

void lcdCaracter(const char lcd01Char, const char linha, unsigned char coluna)	// Envia um caracter: Caracter, Linha, Coluna.
{
	lcdPosicao(linha,coluna);		// Sub rotina para posicionar o cursor (linha e coluna).
	lcd4Bits(lcd01Char,1);			// Chama sub rotina para enviar dados.
}

void lcdTexto(const char *lcd01Texto, const char linha, unsigned char coluna)	// Envia uma String: Texto, Linha, Coluna.
{
//	lcdPosicao(linha,coluna);
    while(*lcd01Texto)				// Executa enquanto diferente de 'null'.
    {
		lcdPosicao(linha,coluna);
		lcd4Bits(*lcd01Texto,1);	// Chama sub rotina para enviar dados.
		lcd01Texto++;				// Incrementa o ponteiro 'buffer'.
		coluna++;
    }
}

void lcdCustom(unsigned char endCGRAM, const unsigned char *lcdDados)	// Caracter do usuario: Endereco(1 a 8), Matriz com os dados.
{
	unsigned char vlrCG, vlrLaco;
	vlrCG = endCGRAM-1;
	vlrCG = (vlrCG<<3);
	endCGRAM =  (vlrCG + 0x40);
	lcd4Bits(endCGRAM,0);
	for(vlrLaco=0;vlrLaco<8;vlrLaco++) lcd4Bits(lcdDados[vlrLaco],1);
	lcd4Bits(0x01,0);
}

/* ---- Teclado ----- */
#ifdef teclado

const unsigned char tabTeclas[]={"147E2580369FABCD"};
unsigned char tecTecOld;		// Variavel local, armazena valor antigo.
unsigned char tecTecNew;		// Variavel local, armazena valor novo.

void tecladoIniciar(void)		// Inicializa o hardware.
{
#ifdef _16F877A					// Se for o PIC16F877A...
	OPTION_REGbits.nRBPU=0;		// Ativa os resistores de Pull-Up no PORTB
    CMCON=0x07;					// Modulo comparador desligado.
	ADCON0 = 0x00;
	ADCON1 = 0x8E;
#endif

#ifdef _16F887					// Se for o PIC16F887...
	OPTION_REGbits.nRBPU=0;		// Ativa os resistores de Pull-Up no PORTB
	ANSELH=0x00;				// Desliga entrada analogica
#endif

#ifdef _18F4550					// Se for o PIC18F4550...
	INTCON2bits.RBPU=0;			// Ativa os resistores de Pull-Up no PORTB
	ADCON1 |= 0x0E;				// Desliga entrada analogica
#endif

	confgTec=0xF0;				// RB3-RB0: Output (Saida=0)  RB7-RB4: Input(Entrada=1)
	saidaTec=0x00;				// Limpa o PORTB.
	tecTecOld=0x00;				// Limpa a variavel local.
	tecTecNew=0x00;				// Limpa a variavel local.
}

unsigned char tecladoLer(void)	// Varre a matriz e retorna o valor encontrado.
{
	unsigned char tecla;
	
	saidaTec=0xF7;
	tecla=saidaTec;
	if(tecla==0xE7) tecTecNew=tabTeclas[0];
	if(tecla==0xD7) tecTecNew=tabTeclas[1];
	if(tecla==0xB7) tecTecNew=tabTeclas[2];
	if(tecla==0x77) tecTecNew=tabTeclas[3];

	saidaTec=0xFB;
	tecla=saidaTec;
	if(tecla==0xEB) tecTecNew=tabTeclas[4];
	if(tecla==0xDB) tecTecNew=tabTeclas[5];
	if(tecla==0xBB) tecTecNew=tabTeclas[6];
	if(tecla==0x7B) tecTecNew=tabTeclas[7];

	saidaTec=0xFD;
	tecla=saidaTec;
	if(tecla==0xED) tecTecNew=tabTeclas[8];
	if(tecla==0xDD) tecTecNew=tabTeclas[9];
	if(tecla==0xBD) tecTecNew=tabTeclas[10];
	if(tecla==0x7D) tecTecNew=tabTeclas[11];

	saidaTec=0xFE;
	tecla=saidaTec;
	if(tecla==0xEE) tecTecNew=tabTeclas[12];
	if(tecla==0xDE) tecTecNew=tabTeclas[13];
	if(tecla==0xBE) tecTecNew=tabTeclas[14];
	if(tecla==0x7E) tecTecNew=tabTeclas[15];
	
	// Codigo anti-repeticao
	if(tecTecNew!=tecTecOld)	tecTecOld=tecTecNew;	// Se o valor atual eh diferente da anterior, salva atual e ... 
	else	tecTecNew=0x00;								// ...se nao... Salva como nao acionada ou nao liberada.						
	return (tecTecNew);									// ...retorna com o valor.
}

#endif

/* ------- DHT 11/22 ------- */
#ifdef dht

#define DHT11	1
#define DHT22	2

unsigned char   timeOut=0;
unsigned int    valor16bRh, valor16bTp;

struct dnthRegistro_t
{
    unsigned char rhInteiro;
    unsigned char rhDecimal;
    unsigned char tempInteiro;
    unsigned char tempDecimal;
    unsigned char checksum;
}dnthRegistro;

void dhtxxIniciar(volatile unsigned char *gate, unsigned char dhtBitVlr) // Inicia o DHTxx: &PORTx, Bit no PORTx (0 a 7).
{    
#define dhtDtIn		bit1(*cfgGate,dhtBit)	// TRISxBits = Entrada
#define dhtDtOut	bit0(*cfgGate,dhtBit)	// TRISxBits = Saida
#define dhtPino1	bit1(*dhtPort,dhtBit)	// PORTxBits = 1
#define dhtPino0	bit0(*dhtPort,dhtBit)	// PORTxBits = 0

	unsigned char *cfgGate, t1, t2;
	extern unsigned char *dhtPort;			// Var externa
	extern unsigned char dhtBit;
	t1 = (char) gate;						// Puxa endereço físico (PORT)
	dhtPort = t1;							// Salva o PORT(int->char)
	t2 = dhtBitVlr;
	dhtBit = t2;
	cfgGate = (t1 + 0x80);					// Carrega o TRISx
	dhtDtOut;								// Programa o pino como saída
	dhtPino0;								// Coloca o pino em '0'
    __delay_ms(20);							// Aguarda 18ms
	dhtPino1;								// Coloca o pino em '1'
    __delay_us(30);							// Aguarda 20us
	dhtDtIn;								// Programa o pino como entrada
}

unsigned char dhtxxLer(void) // Faz leitura e conversao dos bits do DHT.
{
extern unsigned char *dhtPort;				// Var externa
extern unsigned char dhtBit;

#define entraDados	testarBit(*dhtPort,dhtBit)
	
  unsigned char i, k, dado = 0;        // k is used to count 1 bit reading duration
  if(timeOut) return (0);

  for(i = 0; i < 8; i++)
  {
    k = 0;
    while(!(entraDados & 1))                 // Laço que aguarda (subida do sinal) com timeout.
    { 
      k++;
      if(k > 100)
      {
        timeOut = 1;
        break;
      }
      __delay_us(1);
    }
    __delay_us(30);
    
    if(entraDados & 1) dado=((dado<<1) | 1);// Verifica se o dado é '0' ou '1'. Se ativado, agregar o valor.
    else dado=(dado<<1);                    // Senão, apenas desloca um bit a esquerda.
      
      while(entraDados & 1)                 // Laço que aguarda (descida do sinal) com timeout.
      { 
            k++;
            if(k > 100)
            {
                timeOut = 1;
                break;
            }
          __delay_us(1);
      }
    }
  return (dado);
}

unsigned char dhtxxChecar(void) // Verifica pinos d DHT.
{
  __delay_us(40);
//  if(!entraDados)                               // Testa o pino se esta em 0.
	if((testarBit(*dhtPort,dhtBit))==0)
  {
    __delay_us(80);
//    if(entraDados)                               // Testa o pino se esta em 1.
	if((testarBit(*dhtPort,dhtBit))==1)
    {
      __delay_us(50);
      return (1);
    }
  }
  return (0);
}

unsigned char hexDec(unsigned char valor) // Converte valor em Hex para BCD: Valor em Hexa, retorna em BCD.
{
    unsigned cont, tst=1, charconv=0;
    
    for(cont=0;cont<8;cont++)		// Varre o Byte
    {
        if((testarBit(valor,cont))==1) charconv = charconv + tst; // Bit ativado soma o peso matematico.
        tst += tst;	// Gera o peso matematico.
    }
    return (charconv); // Retorna valor em BCD
}

unsigned char dhtxxValor(unsigned char valordht) // Faz leitura dos do DHT escolhido.
{
    unsigned char testar;
    unsigned char temp;
    
    if(valordht<1)valordht=1;
    if(valordht>2)valordht=2;
    
    if(dhtxxChecar())
    {
        dnthRegistro.rhInteiro=dhtxxLer();						// Faz a leitura do 1° grupo de dados e salva.
        dnthRegistro.rhDecimal=dhtxxLer();						// Faz a leitura do 2° grupo de dados e salva.
        dnthRegistro.tempInteiro=dhtxxLer();						// Faz a leitura do 3° grupo de dados e salva.
        dnthRegistro.tempDecimal=dhtxxLer();						// Faz a leitura do 4° grupo de dados e salva.
        dnthRegistro.checksum=dhtxxLer();						// Faz a leitura do 5° grupo de dados e salva.
    }
    
    if(valordht==1)
    {
        temp = hexDec(dnthRegistro.rhInteiro);
        valor16bRh = temp * 10;
        temp = hexDec(dnthRegistro.tempInteiro);
        valor16bTp = temp * 10;
    }
    
    if(valordht==2)
    {
        valor16bRh = (dnthRegistro.rhInteiro<<8) | (dnthRegistro.rhDecimal);
        valor16bTp = (dnthRegistro.tempInteiro<<8) | (dnthRegistro.tempDecimal);
    }
    
    testar=dnthRegistro.rhInteiro+dnthRegistro.rhDecimal+dnthRegistro.tempInteiro+dnthRegistro.tempDecimal;
    if (dnthRegistro.checksum !=testar) return (0);
    else return (1);
}

void dhtxx(unsigned char modelo,unsigned int *humidade,unsigned int *temperatura)
{	
	if(dhtxxValor(modelo))
	{
		*humidade = valor16bRh;
		*temperatura = valor16bTp;
	}
}

#endif // Fim dht

#endif	// BIBLIOTECA_V13_H
/* ------------------ */
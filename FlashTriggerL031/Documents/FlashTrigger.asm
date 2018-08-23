  
;**********************************************************************
;    Filename: FL4411.asm        File Version: 4411
;    Date:                                                            
;    Author: Pavel Janko  jankop@volny.cz                             
;**********************************************************************
;    Files required: <D:\pic4\P12F675.INC>                                                 
;                                                                     
;                                                                     
;**********************************************************************
;    Notes: 
; 031020 Pridan LED blik pri zablesku 
; 031021 Opravena chyba výpoctu limit_delta "sublw" nahrazeno "subwf"
; 031105 Prejmenovani konstanty const5s na const2500ms
; 031220 Ulozeni verze firmware do EEPROM na adresu 0x2110 ve tvaru
;			YY,MM,DD 
;			Zmena konstanty  const2ms  EQU 0x29 na const5ms  EQU 0x64 
;			Snizeni citlivosti limitPGM EQU 0x04 na limitPGM EQU 0x06                                      
; 040314 Pridana funkce Flash_Test k modu PGM
; 040401 Zmena zakladnich algoritmu, pamatuje dva posledni pulsy                                                                  
; 040403 Oprava chyb a zruseni obchvatu pri jednoduchem zablesku 
; 040407 Po testovacim zablesku prechod do stavu zapnuto
;			Pridan test missmach po zablesku v intervalu cca 220 ms
; 040408 Zmena signalu uspesneho PGM na pocet bliknuti
; 			Po testu blesku pridana signalizace LED_BLIK 
; 040411 Zmena rozdelovaci rutiny pro tlacitko
;			Prodlouzeni zablesku LED po blesku na cca 200ms
; 4411	Pridano do ID oznaceni verze                                                               
;**********************************************************************
	list      p=12F675            	; list directive to define processor
	#include <P12F675.INC>  	; processor specific variable definitions
	errorlevel -302
	__CONFIG   _CP_OFF &_CPD_OFF & _WDT_OFF & _MCLRE_OFF &_BODEN_OFF &_PWRTE_ON &_INTRC_OSC_NOCLKOUT 
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
	__idlocs 0x4411 ; Uklada verzi do ctyr ID byte (pouze 4 spodni bity)
						 ; Datum je kodovano hexadecimalne xYxMdDdD
;--------- VARIABLE DEFINITIONS ---------------------------------------
w_temp        	EQU     	0x20	; variable used for context saving 
status_temp   	EQU     	0x21	; variable used for context saving
ramp				EQU		0x22	; filtracni hodnota	
limit				EQU		0x23	; hodnota prahu pro detekci flash
flag				EQU		0x24	; registr priznaku	
counterLO		EQU		0x25	; citac sample x 50mic.sekund
counterHI		EQU		0x26	; 				"
timerLO			EQU		0x27	; citac x 100 ms
timerHI			EQU		0x28	; 				"
butt_tim			EQU		0x29	; citac stisku tlacitka
ucrLO				EQU		0x2a	; universalni registr LO
ucrHI				EQU		0x2b	; universalni registr HI
timer_delay	   EQU		0x2c  ; citac 8bit pro zpozdeni x 50mcs
deltaLO_1		EQU		0x2d  ; Delka intervalu mezi zablesky
deltaHI_1		EQU		0x2e	; ukladaji se posledni a predposledni 
deltaLO_2		EQU		0x2f  ; interval pri programovani
deltaHI_2		EQU		0x30	;				"
deltaLO_3		EQU		0x31	; deltaLO/HI_3 je pomocna promena
deltaHI_3		EQU		0x32	; s nulovou hodnotou		"
delta_newLO		EQU		0x33	; namereny interval mezi zablesky
delta_newHI		EQU		0x34	;				"	
limit_deltaLO  EQU      0x35  ; vypoctova promena pro interval 
limit_deltaHI  EQU      0x36  ; 				"
;--------- CONSTANTS DEFINITIONS---------------------------------------
flag_flash		EQU		0x00	; priznak flash
counter_over	EQU		0x01	; counter for sampling increment to 0x0000
timer_over		EQU		0x02	; counter for sleep decrement to 0xFFFF
delay_over		EQU		0x03	; counter for delay decrement to 0xFF
butt_DW			EQU		0x04	; stisk tlacitka
butt_UP			EQU		0x05	; uvolneni tlacitka
PGM            EQU      0x06  ; flag pro programovani
STD            EQU      0x07  ; flag pro standard
const50mcs		EQU     	0xcd	; konstanta pro TMR0 sample 50mcs
const100ms		EQU     	0xce	; konstanta pro TMR1 sleep/button 100ms
const5ms			EQU		0x64	; konstanta pro 5ms
const2500ms		EQU		0x19	; konstanta pro 2,5s
const13ms		EQU		0xff	; konstanta pro 13ms
limitPGM			EQU		0x06	; citlivost pri programovani
limitSTD			EQU		0x01	; citlivost standardni
sleepLO			EQU		0x50	; LO konstanta pro sleep 18000x100ms=0,5hod
sleepHI			EQU		0x46  ; HI konstanta pro sleep 18000x100ms=0,5hod
EEdeltaLO_1		EQU      0x00  ; EEPROM
EEdeltaHI_1    EQU      0x01  ; EEPROM
EEdeltaLO_2    EQU      0x02  ; EEPROM
EEdeltaHI_2    EQU      0x03  ; EEPROM
flash_OUT      EQU      GPIO0 ; Spinac blesku
button_INP		EQU		GPIO1	; tlacitko proti zemi
led_OUT        EQU      GPIO2 ; LED proti zemi
jumper_INP     EQU      GPIO3 ; vstup rezerva
photo_INP      EQU      GPIO4 ; analogový vstup
power_OUT      EQU      GPIO5 ; napajení fotodiody
;----------------------------------------------------------------------
		ORG     0x000          	; Processor reset vector
reset
errors
		
		goto main           		; Go to beginning of program

		ORG     0x004          	; interrupt vector location

;----------- Obsluha preruseni vcetne ulozeni registru ------------
		movwf w_temp         	; save off current W register contents
		movf STATUS,W        	; move status register into W register
		movwf status_temp     	; save off contents of STATUS register
		bcf STATUS, RP0 			; Restore Bank 0
		btfsc INTCON,T0IF			; test preruseni od timeru sampleru 50mcs
		goto sampler_INT			; preruseni od timeru sampleru 50mcs
		btfsc PIR1,TMR1IF			; test preruseni od timeru casu	100ms
		goto timer1_INT			; preruseni od timeru casu	100ms
		goto errors					; osetreni poruchy vypnutim systemu
;----------------------------------------------------------------------
; 				Obsluha preruseni od TMR0 - 50mcs 
; Kazdym pruchodem je inkrementovan par registru counterLO/HI,
; pokud neni aktivni bit flag<counter_over>. Ten je nastaven, kdyz je 
; counterLO/HI incrementovan na 0x0000. Muze byt rizen i z vnejsku.
; Pri kazdem pruchodu se koriguje hodnota "ramp" na základe okamzité
; hodnoty vzorku. Bit flag<flag_flash> je nastaven, pokud je rozdil mezi
; okamzitym vzorkem  a dlouhodobou hodnotou "ramp" vetsi nez "limit". 
; Bit flag<flag_flash> musi byt nulovan z vnejsku.
;----------------------------------------------------------------------
sampler_INT		
		movlw const50mcs			; konstanta pro 50mcs	
		movwf TMR0					; renastaveni konstanty 50mcs timeru
		movf ramp,W					; ramp -> W
		subwf ADRESH,W				; ADRESH - ramp -> W
		bsf	ADCON0,GO			; start ADC
		btfsc STATUS,Z				; 
		goto nochange_ramp		; odskok pro ADRESH = ramp
		btfss STATUS,C				; 
		goto decr_ramp				; odskok pro ADRESH < ramp
		incf ramp,F					; ramp + 1 -> ramp
		subwf limit,W				; limit - (ADRESH - ramp] -> W
		btfss STATUS,C				; skip pro limit >= (ADRESH - ramp]
		bsf flag,flag_flash		; set flag flash pro limit < (ADRESH - ramp] 
nochange_ramp	incf ramp,F		; ramp = ramp
decr_ramp		decf ramp,F		; ramp - 1 -> ramp
		btfsc flag,counter_over	; preteceni registru -> 
		goto ctstop					; neincrementovat
		incf counterLO,F			;		"
		btfsc STATUS,Z				;		"
		incf counterHI,F			;		"
		btfsc STATUS,Z
		bsf flag,counter_over	;  timer pro flash Roll Over
ctstop	
      btfsc flag,delay_over	; preteceni registru delay
		goto delay_stop			; nedekrementovat
		decf timer_delay,F		; dekrement registru delay
		btfsc STATUS,Z				
		bsf flag,delay_over		; prosel nulou nastav priznak
delay_stop
		bcf INTCON,T0IF			; zruseni priznaku preruseni od timeru0
reti_end			
		movf status_temp,W		; retrieve copy of STATUS register
		movwf STATUS				; restore pre-isr STATUS register contents
		swapf w_temp,F
		swapf w_temp,W				; restore pre-isr W register contents
		retfie						; return from interrupt
;----------------------------------------------------------------------
; Rutina pro preruseni od casovace TMR1. Preruseni se opakuje po 100ms.
; Podprogram meri dobu od zapnuti pro vypnuti systemu - sleep.
; Zajistuje take cteni funkcniho tlacitka. Kazdym pruchodem je
; dekrementovan par registru timerLO/HI, pokud neni aktivni
; bit flag<timer_over>. Ten je nastaven, kdyz je timerLO/HI dekrementovan
; na 0xFFFF. Muze byt rizen i z vnejsku. Bit flag<butt_UP>
; signalizuje uvolneni tlacitka po stisku. Doba stisku tlacitka je 
; v registru butt_tim. Pri dosazeni hodnoty 0xff tj. 25,5s 
; se zastavi - neprejde do 0.
;----------------------------------------------------------------------
timer1_INT	
		movlw const100ms			; Value to load into TMR1H - konstanta pro 100ms
		movwf TMR1H					; Write High byte - konstanta to TMR1
		btfsc flag,butt_UP		; butt_UP=1  neprovadet test tlacitka 
		goto timer_incr			; pouze time increment
		btfss GPIO,button_INP	; test tlacitka
		goto butt_tim_inc			; ANO, tlacitko je stisknuto
		btfsc flag,butt_DW		; NE, bylo tlacitko pred uvolnenim stisknuto?
		bsf flag,butt_UP			; ANO, nastav priznak akce tlacitka
		bcf flag,butt_DW			; zrus priznak stisku tlacitka
timer_incr
		btfsc flag,timer_over	;preteceni registru -> 
		goto not_inc_timer		; nedekrementovat
		movlw 0x01					; timer pro sleep
		subwf timerLO,F			;		"
		btfss STATUS,C				;     "
		subwf timerHI,F			;		"
		btfss STATUS,C				;     "
		bsf flag,timer_over		; timer pro SLEEP Roll Over
not_inc_timer		
		bcf PIR1,TMR1IF			; mazani priznaku preruseni TMR1 
		goto reti_end
butt_tim_inc
		bsf flag,butt_DW			; priznak stisku tlacitka
		incf butt_tim,F			; increment citac pro button
		btfsc STATUS,Z				; pri preteceni citace ho
		decf butt_tim,F			; vrat na 0xFF
		goto timer_incr
;---------------------------------------------------------------------
; Priprava procesoru na SLEEP, odpojeni spotrebicu a povoleni probuzeni
; od tlacitka
;---------------------------------------------------------------------
main	bcf INTCON,GIE				; zakaz vsech preruseni	
		call reinit_CPU			; odstaveni periferii a procesu
      call set_IO             ; nastaveni portu
      bcf GPIO,power_OUT      ; vypnuti fotodiody
loop	
		clrf  ucrLO					; Delay 100 ms pred usnutim
		movlw 0x82	
		movwf ucrHI
delay	
		btfss GPIO,button_INP	; cteni tlacitka pred usnutim
		goto loop					; tlacitko neni uvolneno
		decfsz ucrLO,F				;			"
		goto delay					;			"
		decfsz ucrHI,F				;			"	
		goto delay					;			"
		bsf STATUS,RP0 			; Bank 1
		bsf IOCB,button_INP		; maska IRQ pro tlacitko
		bcf STATUS,RP0 			; Bank 0
		movf GPIO,W					; nacteni referencni hodnoty vstupu
		bsf INTCON,GPIE			; povolení wake-up od zmeny vstupu - tlacitka
		sleep
;----------------------------------------------------------------------
; Probuzeni procesoru po SLEEP, reinicializace
;----------------------------------------------------------------------
		bcf STATUS,RP0 			; Bank 0
		clrf  ucrLO			      ; Delay 50 ms po probuzeni
		movlw 0x41					;			"
		movwf ucrHI					;			"
delay2								;			"
		decfsz ucrLO,F				;			"
		goto delay2					;			"
		decfsz ucrHI,F				;			"
		goto delay2					;			"
		btfsc GPIO,button_INP	; cteni tlacitka po probuzeni
		goto errors					; tlacitko nestisknuto -> SLEEP
		bsf STATUS, RP0 			; Restore Bank 1
		call 0x3FF 					; Get the cal value
		movwf OSCCAL 				; Calibrate
		bcf STATUS,RP0 			; Bank 0
		bsf flag,butt_DW			; pre press pro preruseni (pouze po sleep)
		bcf flag,butt_UP
      bcf flag,STD            ; nulovani priznaku po Sleeep
		clrf butt_tim				; mazani citace tlacitka 25s
		call reinit_CPU			; zastaveni periferii a procesu
		call set_IO					; nastaveni periferii
		call set_ADC				; nastaveni prevodniku ADC
		call set_TMR1				; nastaveni casovace TMR1
		call set_TMR0				; nastaveni casovace TMR0
		call set_IRQ				; povoleni IRQ od TMR0/TMR1
;-----------------------------------------------------------------------
;							Rozdelovaci rutina
;-----------------------------------------------------------------------
start
      bcf flag,PGM				; zrus priznak odskoku z PGM 
      call read_EEPROM        ; ctení zaznamu EEPROM do RAM
      btfss flag,STD				; IF STD - neblikat !
      call LED_BLIK				; ELSE zablikat pocet jako po zapnuti
		btfsc flag,STD				; IF STD - LED_DW
      call LED_DW
wait_butt
		movlw const2500ms			; konstanta 2,5s
		subwf butt_tim,W			; test na cas. limit 2,5s
		btfsc STATUS,C				; IF T > 2.5s
		bsf flag,PGM				; THEN flag PGM
      btfsc flag,PGM				; IF flag PGM
      bsf GPIO,led_OUT		   ; THEN LED ON
		btfss flag,butt_UP		; cekej na uvolneni tlacitka
		goto wait_butt
		clrf butt_tim				; mazani citace tlacitka
		bcf flag,butt_UP			; mazani priznaku uvolneni tlacitka
      btfsc flag,PGM				; IF flag PGM
      goto PGM_start				; THEN Goto PGM
		btfss flag,STD				; IF flag STD (navrat z STD)
      goto STD_start				; ELSE Goto STD
		goto reset					; THEN Sleep
			
shut_down
		call LED_DW
		goto reset
		
;---------------------------------------------------------------
;                 Hlavni smycka
;---------------------------------------------------------------
STD_start
		call read_EEPROM
		clrf deltaLO_3
		clrf deltaHI_3
;----------------- Nastaveni prahu citlivosti pro STD ----------
		movlw limitSTD				; konstanta citlivosti pro STD
		movwf limit					; nastaveni citlivosti pro STD
;----------------- Osetreni tlacitka --------------------------
		clrf butt_tim				; mazani citace tlacitka
		bcf flag,butt_UP			; mazani priznaku uvolneni tlacitka
      bcf flag,PGM				; mazani priznaku PGM				
      bsf flag,STD				; nastaveni priznaku STD
;----------------- Nastaveni casovace pro SLEEP za 30 min ------
after_flash
	   bsf flag,timer_over		; zastav odcitani 100ms pro sleep 1min
      movlw sleepLO				; nastaveni odcitani 30 min
	   movwf timerLO				; 1800x100ms
		movlw sleepHI 				;	"
		movwf timerHI				;	"
		bcf flag,timer_over		; start sleep pro STD
;----------------- Prodleva pro ustaleni ss hodnoty ------------
		bsf flag,delay_over		; delay stop
		movlw const13ms			; do citace konstanta pro cca 13ms
		movwf timer_delay	   	; do citace konstanta pro cca 13ms
		bcf flag,delay_over		; delay start
adelay_13ms
		btfss flag, delay_over	; cekej 13ms na ustaleni ramp
		goto adelay_13ms			; cekej 13ms na ustaleni ramp
repeat_25
;------------------ Reset parametru zablesku -------------------
   	bsf flag,counter_over	; stop citace 50mcs
	   clrf counterLO				; priprav counter na mereni doby- 
		clrf counterHI				; mezi zablesky,do citace dej 0x0000
		movlw deltaLO_1			; pointer na zaèátek pole
		movwf FSR					; do index registru
		bcf flag,flag_flash		; nuluj priznak zablesku
;------------------ Cekani na prvni zablesk --------------------
await_first_flash
		bcf flag,butt_UP
		btfsc flag,butt_DW		; stisk tlacitka?
		goto start				   ; ANO bez na rozdeleni
		btfsc flag, timer_over	; sleep po 30 min
		goto shut_down				; ANO vypni
		btfss flag,flag_flash	; prvni zablesk?
		goto await_first_flash	; neni prvni zablesk
anexting		
;------------------ Ulozeni doby mezi zablesky -----------------
      bsf flag,counter_over	; stop citace 50mcs pro cteni
     	movf counterLO,W
      movwf delta_newLO
      movf counterHI,W
      movwf delta_newHI
;------------------ Aktivace citace intervalu -------------------
	  	clrf counterLO				; priprav counter na mereni doby-
	  	clrf counterHI				; mezi zablesky 0x0000
	  	bcf flag,counter_over	; start citace 50mcs
;------------------- Nastaveni prodlevy pro kliks --------------
		bsf flag,delay_over		; stop delay 
		movlw const5ms				; konstanta pro cca 5ms
		movwf timer_delay			; do citace konstanta pro cca 2ms
		bcf flag,delay_over		; start delay
;-------------------- Vypocet tolerancnich mezi ----------------
		movf INDF,w					; vem z tabulky interval
		movwf limit_deltaLO
		incf FSR,F
		movf INDF,w					; limit_deltaHI do W   =X
		movwf limit_deltaHI
		subwf delta_newHI,W     ; delta_newHI - limit_deltaHI (YHI - XHI)
   	btfss STATUS,Z
	   goto aresults
		movf limit_deltaLO,w 	; X
		subwf delta_newLO,w	   ; delta_newLO - limit_deltaLO (YLO - XLO)
aresults                      ; if X=Y then now Z=1.
      btfss STATUS,C          ; if X<=Y then now C=1
      goto adeltabiger        ; if Y<X then now C=0
;--------------------------------------------------------------------
      movf delta_newLO,W		; kopie DELTA_NEW od UCR
		movwf ucrLO
		movf delta_newHI,W
		movwf ucrHI
;--------------------------------------------------------------------
		movf limit_deltaLO,W		; (Y-X)-> UCR
		subwf ucrLO,F
		movf limit_deltaHI,W
		btfss STATUS,C
		incf limit_deltaHI,W
		subwf ucrHI,F
		goto aendcomp
;--------------------------------------------------------------------		
adeltabiger
		movf limit_deltaLO,W			; kopie limit_dELTA od UCR
		movwf ucrLO
		movf limit_deltaHI,W
		movwf ucrHI
;--------------------------------------------------------------------
		movf delta_newLO,W      	; (X-Y)-> UCR
		subwf ucrLO,F
		movf delta_newHI,W
		btfss STATUS,C
		incf delta_newHI,W
		subwf ucrHI,F           	; v UCR je ABS(DELTA_NEW-LIMIT_DELTA)
aendcomp
		bcf STATUS,C					; polovina limit_delta 
		rrf limit_deltaHI,F
		rrf limit_deltaLO,F  
	   movf ucrHI,W           		; X
	   subwf limit_deltaHI,W		; (YHI - XHI)
   	btfss STATUS,Z
	   goto aresults2
		movf ucrLO,W            	; X
		subwf limit_deltaLO,W 		; (YLO - XLO)
aresults2                     	; if X=Y then now Z=1.
      btfsc STATUS,C          	; if X<=Y then now C=1
      goto ainlimit           	; if Y<X then now C=0
      movlw deltaLO_1				; velka odchylka, pointer na zaèátek pole
		movwf FSR						; 
ainlimit
		btfss STATUS,C          	; if X<=Y then now C=1
		goto aonlyfirst
		incf FSR,F						; pri in-limit zvys ukazatel pole
aonlyfirst 
;------------------- Odpaleni blesku --------------------------
		movf INDF,w
		movwf limit_deltaLO
		incf FSR,F
		movf INDF,w
		decf FSR,F
		iorwf limit_deltaLO,w	; test nuly pro okamzity zablesk
		btfss STATUS,Z
		goto adelay0_2ms
		bsf GPIO,flash_OUT	   ; zapnutí spinace blesku
      bsf GPIO,led_OUT			; zapnutí LED ON
;------------------- Nastaveni 13ms pro ridici impuls ---------
		bsf flag,delay_over		; delay stop
		movlw const13ms			; do citace konstanta pro cca 13ms
		movwf timer_delay			; do citace konstanta pro cca 13ms
		bcf flag,delay_over		; delay start
flash_13ms
		btfss flag, delay_over	; cekej 13ms - impuls pro blesk
		goto flash_13ms			;
      bcf GPIO,flash_OUT	   ; vypnuti spinace blesku
;--------------------missing flash-----------------------------
		bcf flag,flag_flash		; nuluj zablesk		
		bsf flag,counter_over	; stop citace 50mcs
		movlw 0x5f					; nastaveni konstant timeru na 0,200s
		movwf counterLO			;		"
		movlw 0xf0					; 		"
		movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
cek_flash		
		btfss flag,counter_over ;
		goto cek_flash
		bcf GPIO,led_OUT		   ; vypnuti LED 
		btfss flag,flag_flash	; TEST zablesku
		goto after_flash
		call signal02
		goto after_flash
;------------------- Smycka prodlevy pro kliks ----------------
adelay0_2ms
		btfss flag, delay_over	; cekej 2ms po prechodnem deji
		goto adelay0_2ms		   ; cekej 2ms po prechodnem deji
		bcf flag,flag_flash		; nuluj zablesk
;------------------- Cekani na dalsi blesk ---------------------
await_next_flash
      btfsc flag,counter_over ; uplynulo 2.5s od -
      goto repeat_25          ; posledniho zablesku
		btfss flag,flag_flash	; TEST zablesku
		goto await_next_flash	; REPT cekej na zablesk
		goto anexting

;---------------------------------------------------------------
;                    Programovani
;---------------------------------------------------------------
PGM_start
;----------------- Nastaveni prahu citlivosti pro PGM ----------
		movlw limitPGM				; konstanta citlivosti pro PGM
		movwf limit					; nastaveni citlivosti pro PGM
;----------------- Osetreni tlacitka --------------------------
		clrf butt_tim				; mazani citace tlacitka
		bcf flag,butt_UP			; mazani priznaku uvolneni tlacitka
      bsf flag,PGM
      bcf flag,STD
;----------------- Nastaveni casovace pro SLEEP za 1 min ------
	   bsf flag,timer_over		; zastav odcitani 100ms pro sleep 1min
      movlw 0x58     			; nastaveni odcitani 1 min
	   movwf timerLO				; 600x100ms
		movlw 0x02  				;	"
		movwf timerHI				;	"
		bcf flag,timer_over		; start sleep pro PGM
;----------------- Prodleva pro ustaleni ss hodnoty ------------
		bsf flag,delay_over		; delay stop
		movlw const13ms			; do citace konstanta pro cca 13ms
		movwf timer_delay			; do citace konstanta pro cca 13ms
		bcf flag,delay_over		; delay start
delay_13ms
		btfss flag, delay_over	; cekej 13ms na ustaleni ramp
		goto delay_13ms			; cekej 13ms na ustaleni ramp
;------------------ Reset parametru zablesku -------------------
   	bsf flag,counter_over	; stop citace 50mcs
	   clrf counterLO			   ; priprav counter na mereni doby- 
		clrf counterHI			   ; mezi zablesky,do citace dej 0x0000
		clrf deltaLO_1				; reset DELTA, interval zablesku-
		clrf deltaHI_1				; na hodnotu 0x0000
;		clrf deltaLO_2				; 
;		clrf deltaHI_2				; 
		bcf flag,flag_flash		; nuluj priznak zablesku
;------------------ Cekani na prvni zablesk --------------------
wait_first_flash
		btfsc flag,butt_DW		; stisk tlacitka?
		goto test_flash			; Testovaci zablesk a vypnuti
		btfsc flag, timer_over	; sleep po 1 min
		goto shut_down				; ANO vypni
		btfss flag,flag_flash	; prvni zablesk?
		goto wait_first_flash	; neni prvni zablesk
nexting		
;------------------ Ulozeni doby mezi zablesky -----------------
      bsf flag,counter_over	; stop citace 50mcs pro cteni
      movf deltaLO_1,W			; posuv v zasobniku
      movwf deltaLO_2			; z delta_1 do delta_2
		movf deltaHI_1,W
      movwf deltaHI_2
      movf counterLO,W
      movwf deltaLO_1
      movf counterHI,W
      movwf deltaHI_1
;------------------ Aktivace citace intervalu -------------------
	   clrf counterLO			   ; priprav counter na mereni doby-
		clrf counterHI			   ; mezi zablesky 0x0000
		bcf flag,counter_over	; start citace 50mcs
;------------------- Nastaveni prodlevy pro kliks --------------
		bsf flag,delay_over		; stop delay 
		movlw const5ms				; konstanta pro cca 5ms
		movwf timer_delay			; 
		bcf flag,delay_over		; start delay
;------------------- Smycka prodlevy pro kliks ----------------
delay0_2ms
		btfss flag, delay_over	; cekej 5ms po prechodnem deji
		goto delay0_2ms			; 
		bcf flag,flag_flash		; nuluj zablesk
;------------------- Cekani na dalsi blesk ---------------------
wait_next_flash
      btfsc flag,counter_over ; uplynulo 2.5s od posledniho-
      goto uloz_data          ; zablesku
		btfss flag,flag_flash	; TEST zablesku
		goto wait_next_flash		; REPT cekej na zablesk
		goto nexting

uloz_data
;------------ Zapis dat do EEPROM ---------------------
		movf deltaLO_2,w			; test nuly ve druhém byte
		iorwf deltaHI_2,w
		btfsc STATUS,Z				; je nula
		goto notrans				; nech stack být
		movf deltaLO_1,W			; posuv v zasobniku
      movwf ucrLO					; z delta_2 do delta_1
		movf deltaHI_1,W
      movwf ucrHI
      movf deltaLO_2,W
      movwf deltaLO_1
      movf deltaHI_2,W
      movwf deltaHI_1
      movf ucrLO,W
      movwf deltaLO_2
      movf ucrHI,w
      movwf deltaHI_2
notrans
      bcf INTCON,GIE          ; Disable INTs
      bsf STATUS,RP0          ; Bank 1
      movlw EEdeltaLO_1			; pocatek EEPROM - 
      movwf ucrLO             ; do pomocne promene.
      movlw deltaLO_1			; pocatek ukladaneho pole -
      movwf FSR               ; do indexregistru.
nextbyte
      movf ucrLO,W            ; Adresa v EEPROM do
      movwf EEADR             ; EEADR
      movf INDF,W             ; data do -
      movwf EEDATA            ; EEDATA
      bsf EECON1,WREN         ; Enable write
      movlw 0x55              ; Unlock write
      movwf EECON2            ;
      movlw 0xAA              ;
      movwf EECON2            ;
      bsf EECON1,WR           ;Start the write
endwrite
      btfsc EECON1,WR
      goto endwrite
;--------------------- Verify --------------------------      
      movf EEDATA,W           ; EEDATA not changed from previous write
      bsf EECON1,RD           ; YES, Read the value written
      xorwf EEDATA,W
      btfss STATUS,Z          ; Is data the same
      goto errors             ; No, handle error
      incf FSR,F              ; Yes, continue
      incf ucrLO,F
      movlw 0x04
      subwf ucrLO,W
      btfss STATUS,Z
      goto nextbyte
      bcf STATUS,RP0          ; Bank 0
      bcf PIE1,EEIF           ;
      bsf INTCON,GIE          ; Enable INTS
		clrf butt_tim				; mazani citace tlacitka
		bsf flag,butt_UP			; nastaveni priznaku uvolneni tlacitka
		call pausa	
		call pausa
		goto start

;------------------ Prevzeti parametru z pameti EE -------------
read_EEPROM
		bsf STATUS,RP0 			; Bank 1
		movlw EEdeltaLO_1	   	; adresa v EEPROM
		movwf EEADR 			   ; Address to read
		bsf EECON1,RD 			   ; EE Read
		movf EEDATA,W 			   ; Move
		movwf deltaLO_1		   ; deltaLO_1=EEdeltaLO_1
		movlw EEdeltaHI_1	   	; adresa v EEPROM
		movwf EEADR 			   ; Address to read
		bsf EECON1,RD 			   ; EE Read
		movf EEDATA,W 			   ; Move
		movwf deltaHI_1		   ; deltaHI_1=EEdeltaHI_1
		movlw EEdeltaLO_2 		; adresa v EEPROM
		movwf EEADR 			   ; Address to read
		bsf EECON1,RD 			   ; EE Read
		movf EEDATA,W 			   ; Move
		movwf deltaLO_2		   ; deltaHI_2=EEdeltaHI_2
		movlw EEdeltaHI_2 		; adresa v EEPROM
		movwf EEADR 			   ; Address to read
		bsf EECON1,RD 			   ; EE Read
		movf EEDATA,W 			   ; Move
		movwf deltaHI_2		   ; deltaHI_2=EEdeltaHI_2
		bcf STATUS,RP0 			; Bank 0
      return

;-----------------------------------------------------------------------
; Zastaveni cinnosti periferii a IRQ pro SLEEP a reinicializace po SLEEP
;-----------------------------------------------------------------------
reinit_CPU
		bcf STATUS, RP0 			; Restore Bank 0
		clrf INTCON					; zakaz vsech IRQ
		clrf T1CON 					; Stop Timer1,LP oscillator is off,Internal clock(F OSC /4)
		clrf ADCON0					; vypnuti ADC - snizeni spotreby
		movlw 0x07					; odpojeni komparatoru CMP
		movwf CMCON					;       "
		bsf STATUS,RP0 			; Bank 1
		clrf PIE1					; zakaz vsech IRQ periferii
		clrf OPTION_REG			; TMR0 internal clock source,prescaler to the TIMER0
		clrf VRCON					; vypnuti reference - snizeni spotreby
		clrf ANSEL					; odpojeni ADC
		bcf STATUS, RP0 			; Restore Bank 0
		clrf INTCON					; mazani priznaku IRQ
		clrf PIR1					; mazani priznaku IRQ periferii
		return
;----------------------------------------------------------------------
; Nastavení vstupu a vystupu.
;----------------------------------------------------------------------
set_IO
		bcf STATUS,RP0 			; Bank 0
		bcf INTCON,GPIE			; Disables the GPIO port change interrupt
		clrf GPIO					; reset outputs for future
		bsf STATUS,RP0 			; Bank 1
      clrf IOCB					; zakaz vsech INTERRUPT-ON-CHANGE
      clrf WPU						; Pull-up disabled for all inputs
      bsf WPU,button_INP      ; Pull Up povolen pro vstup tlaèítka
		bcf OPTION_REG,NOT_GPPU	; GPIO pull-ups are enabled
		bsf TRISIO,photo_INP		; As input ADC for photodiode 
		bsf TRISIO,button_INP	; As input for button
      bsf TRISIO,jumper_INP   ; Rezerva pr GPIO3
      bcf TRISIO,power_OUT		; As output for drive photodiode
		bcf TRISIO,led_OUT		; As output for LED driver
		bcf TRISIO,flash_OUT	   ; As output for flash driver
		bcf STATUS, RP0 			; Restore Bank 0
		bsf GPIO,power_OUT		; output for drive photodiode to "HI"
		bcf GPIO,led_OUT			; output for LED drive to "LO"
		bcf GPIO,flash_OUT		; output for flash drive to "LO"
		return
;----------------------------------------------------------------------
; Nastavení ADC
;----------------------------------------------------------------------
set_ADC
		bsf STATUS, RP0 			; Bank 1
		bcf PIE1,ADIE				; dissable ADC IRQ
		bsf ANSEL,ANS3				; select analog input ANS3(GPIO4) for photodiode
		bsf ANSEL,ADCS0			; A/D Conversion Clock Select F OSC /8
		bcf STATUS, RP0 			; Bank 0
		bcf ADCON0,ADFM			; A/D Result Formed Select bit 0=left justified	
		bcf ADCON0,VCFG			; Voltage Reference bit 0=Vdd
		bsf ADCON0,CHS0			; Analog Channel Select bits = 11
		bsf ADCON0,CHS1			; 11 = Channel 03 (AN3)
		bsf ADCON0,ADON			; Start ADC
		return
;-----------------------------------------------------------------------
;				Povoleni preruseni TMR0 a TMR1
;-----------------------------------------------------------------------
set_IRQ		
		bsf INTCON,PEIE			; Enables all unmasked peripheral interrupts
		bsf INTCON,GIE				; Enables all unmasked interrupts
		return
;----------------------------------------------------------------------
;		Timer1 Initialization segment with run and start its interrupt only
;----------------------------------------------------------------------
set_TMR1
		clrf T1CON 					; Stop Timer1,LP oscillator is off,Internal clock (F OSC /4)
		bsf T1CON,T1CKPS1			; prescaler /8
		bsf T1CON,T1CKPS0			; prescaler /8
		bsf STATUS,RP0 			; Bank 1
		bcf PIE1,TMR1IE			; Disables the TMR1 overflow interrupt
		bcf STATUS,RP0 			; Bank 0
		bcf PIR1,TMR1IF 			; clear TMR1 IRQ flag - register overflowed
		clrf TMR1L 					; set Timer1 Low byte register to zero
		movlw const100ms			; set Timer1 High byte register
		movwf TMR1H					; Write High byte - konstanta to TMR1
		bsf STATUS,RP0 			; Bank 1
		bsf PIE1,TMR1IE			; Enables the TMR1 overflow interrupt
		bcf STATUS,RP0 			; Bank 0
		bsf T1CON,TMR1ON 			; Timer1 starts to increment
		return
;----------------------------------------------------------------------
;		Timer0 Initialization segment with run and start its interrupt only
;----------------------------------------------------------------------
set_TMR0
		bsf STATUS,RP0 			; Bank 1
		bsf OPTION_REG,T0CS		; Clock source GP2/T0CKI pin "Stop TMR0"
		bsf OPTION_REG,PSA		; Prescaler for WDT (OFF) Rate 1:1
		bcf STATUS,RP0 			; Bank 0
		bcf INTCON,T0IE			; Disables the TMR0 overflow interrupt
		bcf INTCON,T0IF 			; clear TMR0 IRQ flag - register overflowed
		movlw const50mcs			; set Timer0  register
		movwf TMR0					; set TMR0 constant
		bsf STATUS,RP0 			; Bank 1
		bcf OPTION_REG,T0CS		; Internal instruction cycle clock "Start TMR0"
		bcf STATUS,RP0 			; Bank 0
		bsf INTCON,T0IE			; enables the TMR0 overflow interrupt
		return
;----------------------------------------------------------------------
;	Podprogram pro pomale rozsvecovani zhasinani LED
;----------------------------------------------------------------------
LED_DW
		clrf ucrLO
loopled
		movf ucrLO,W
		movwf ucrHI
lole0	goto hup1
hup1	goto hup2
hup2	bcf GPIO,led_OUT        ; LED OFF
		incfsz ucrHI,F
		goto lole0
		movwf ucrHI
		comf ucrHI,F
lole1	goto hup3
hup3	goto hup4
hup4	bsf GPIO,led_OUT        ; LED ON
		incfsz ucrHI,F
		goto lole1
		incfsz ucrLO,F
		goto loopled
;-------------------------------------------
		clrf ucrLO
loopledoff
		movf ucrLO,W
		movwf ucrHI
lole0off
		goto hup5
hup5	goto hup6
hup6	bsf GPIO,led_OUT        ; LED ON
		incfsz ucrHI,F
		goto lole0off
		movwf ucrHI
		comf ucrHI,F
lole1off
		goto hup7
hup7	goto hup8
hup8	bcf GPIO,led_OUT  	   ; LED OFF
		incfsz ucrHI,F
		goto lole1off
		incfsz ucrLO,F
		goto loopledoff
      return                  ; LED je vypnuta

;------------------------------------------------------
;        Signal Missing Flash
;------------------------------------------------------
signal02
      movlw 0x14              ; pocet zablesku
      movwf ucrLO             ; pomocna promena cyklu
mvrtto
      
      bsf GPIO,led_OUT		   ; LED ON 
      bsf flag,counter_over	; stop citace 50mcs
		movlw 0x8e					; nastaveni konstant timeru 
		movwf counterLO			;		"
		movlw 0xfd					; 		"
		movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
mloop672  
      
      btfss flag,counter_over  
	   goto mloop672
      bcf GPIO,led_OUT			; LED OFF
      bsf flag,counter_over	; stop citace 50mcs
		movlw 0x8e					; nastaveni konstant timeru 
		movwf counterLO			;		"
		movlw 0xfd			   	; 		"
		movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
mloop677  
      btfss flag,counter_over  
	   goto mloop677
      decfsz ucrLO,F
      goto mvrtto              ; opakuj bliknuti
      return
;----------------- Signalizace pri zapnuti ---------
LED_BLIK
		call one_LED_BLIK
      movf deltaLO_1,W
      iorwf deltaHI_1,W
      btfss STATUS,Z
      call one_LED_BLIK       
		movf deltaLO_2,W
      iorwf deltaHI_2,W
      btfss STATUS,Z
      call one_LED_BLIK
      return
;-----------Jedno bliknuti LED ------------------------------- 
one_LED_BLIK
		bsf flag,counter_over	; stop citace 50mcs
		movlw 0x77					; nastaveni konstant timeru na 0,25s
		movwf counterLO			;		"
		movlw 0xec					; 		"
	   movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
      bsf GPIO,led_OUT			; LED ON
wait_A
      btfss flag,counter_over ; Ceka 0,25s 
		goto wait_A
pausa            
		bsf flag,counter_over	; stop citace 50mcs
		movlw 0x77					; nastaveni konstant timeru na 0,25s
		movwf counterLO			;		"
		movlw 0xec					; 		"
		movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
		bcf GPIO,led_OUT			; LED OFF
wait_B
      btfss flag,counter_over ; Ceka 0,25s 
		goto wait_B
      return
            
;-----------Test Flash------------------------------------------
test_flash							
		bcf GPIO,led_OUT			; LED OFF
		bsf flag,counter_over	; stop citace 50mcs
		movlw 0x77					; nastaveni konstant timeru na 0,25s
		movwf counterLO			;		"
		movlw 0xec					; 		"
	   movwf counterHI			;		"
		bcf flag,counter_over	; start citace 50mcs
wait_night
      btfss flag,counter_over ; Ceka 0,25s 
		goto wait_night         ;		"
		bsf GPIO,flash_OUT	   ; zapnutí spinace blesku
      bsf GPIO,led_OUT		   ; zapnutí LED ON
;------------------- Nastaveni 13ms pro ridici impuls ---------
		bsf flag,delay_over		; delay stop
		movlw const13ms			; do citace konstanta pro cca 13ms
		movwf timer_delay		   ; do citace konstanta pro cca 13ms
		bcf flag,delay_over		; delay start
flash_test_wait
		btfss flag, delay_over	; cekej 13ms - impuls pro blesk
		goto flash_test_wait		;
      bcf GPIO,led_OUT		   ; vypnuti LED 
		bcf GPIO,flash_OUT	   ; vypnuti spinace blesku
		call pausa
		call pausa
		goto start     
		END                     ; directive 'end of program'


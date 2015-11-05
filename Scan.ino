#include <arduino2.h>

//#include <Encoder.h>
//#include "EncoderH.h"
#include <TimerOne.h>
#include <TimerThree.h>
//int pinEncoder[4][2]={{0,0}{1,1}{2,2}{3,3}}; 
int pinEncoder1[4]={14,24,34,44}; //распиновка условная. Нужно менять при сборке. Каждый двигатель должен соответствовать своему энкодеру.
int pinEncoder2[4]={15,25,35,45};
//pinEngine[4][3]={{0,0,0}{1,1,1}{2,2,2}{3,3,3}};
int engPinStep[4]={10,20,30,40};
int engPinDir[4]={11,21,31,41};
int engPinEn[4]={12,22,32,42};
int pinOpen[4] = {1,2,3,18};



//int pinStep = 8;
//int pinDIR = 6;



int inSize= 0; // Переменная которая будет содержать размер буфера
char str[256]; // Так как типа string тут нет, будем использовать массив символов
int valuePinOpen = 0;
int valuePinClose = 0;
boolean er=false;
byte axis; // ось. 0-ось не выбрана.
long int par[5];
long int destination[4]={0,0,0,0}; //конечная точка
boolean forwardDiretion[4]={1,1,1,1};
boolean tagFind[4]={0,0,0,0};


class EncoderH
{	
	// Переменные-члены класса
	// Устанавливаются при запуске
	int encPin[2]; // Номер пина с энкодером
	byte number; //номер энкодера
	//long oldPos; //предыдущая позиция
	//Encoder myEnc; //объект encoder
	
		// Конструктор - создает EncoderH
	// и инициализирует переменные-члены
	// и состояние
	public:
    boolean prevLevel[2]; //предыдущее состояние пинов
    boolean level[2]; //текущее состояние пинов
    long int pos; //текущая позиция
	EncoderH(int pin1, int pin2, byte num)
	{
		encPin[0] = pin1;
		encPin[1] = pin2;
		number = num; //номер энкодера в соответствии с номером двигателя
		pinMode2(encPin[0], INPUT); 
   
   
		pinMode2(encPin[1], INPUT);
		prevLevel[0] = digitalRead2(encPin[0]);     // считываем состояние 1 выхода энкодера 
		prevLevel[1] = digitalRead2(encPin[1]);     // считываем состояние 2 выхода энкодера  
		//oldPos = sOldPos;
	}
	// Текущее состояние

	void Update()
	{
		level[0] = digitalRead2(encPin[0]);     // считываем состояние 1 выхода энкодера 
		level[1] = digitalRead2(encPin[1]);     // считываем состояние 2 выхода энкодера  
		if((!level[0]) && (prevLevel[0])){    // если состояние изменилось с положительного к нулю
			if(level[1]) 
			{
				// выход В в полож. сост., значит вращение по часовой стрелке
				pos++;              
			}   
			else 
			{
				// выход В в 0 сост., значит вращение против часовой стрелки     
				pos--;              
			} 
		}   
		prevLevel[0]=level[0];     // сохраняем значение 1 для следующего цикла 
	}
};

class Engine
{
	
	
	//long destination; //конечная точка
	//int engPin[3]; //пины
	int stepPin;
	int dirPin;
	int enPin;
	int engState;
	//int mDuration; //продолжительность движения
	//int sDuration; //продолжительность остановки
	byte number; //номер двигателя
	unsigned long previousMillis;
	unsigned long currentMillis; // текущее время в миллисекундах

	public: 
    byte stat; //битовое поле состояния
    boolean power; //питание (state)
    byte erCode; //код ошибки
	boolean inv;
  
	Engine(int pin1, int pin2, int pin3, byte num, boolean invert=0, byte st=0, byte erC=0) //распиновка, номер двигателя,направление, состояние, код ошибки
	{
		stepPin = pin1;
		dirPin = pin2;
		enPin = pin3;
		number = num;
		pinMode2(stepPin, OUTPUT); 
		pinMode2(dirPin, OUTPUT);
		pinMode2(enPin, OUTPUT);
		previousMillis = millis();
		SetPower(0);
		stat = st;
		erCode = erC;
		power=0;
		inv=invert;
	}


	void SetPower(boolean p) //установка питания
	{
		power = p; 
		if(p) digitalWrite2(enPin, HIGH);
		else digitalWrite2(enPin, LOW);
	}
	void SetErCode(byte erC) //установка кода ошибки
	{
		stat = 1;
		erCode = erC;
	}
	void SetStat(byte st) //установка состояния двигателя
	{
		stat = st;
	}
	void Move(boolean forDir, float m=0.1, float s=0.1) //направление (true = вперед, false = назад), задержки заданы по-умолчанию, но возможно переназначение по ситуации. 
	{ 
		forDir=forDir ^ inv;
		if(forDir) digitalWrite2(dirPin, LOW); 
		else digitalWrite2(dirPin, HIGH); 
		currentMillis = millis(); // текущее время в миллисекундах 
		if(engState == HIGH) 
		{   
			engState = LOW; // выключаем 
			digitalWrite2(stepPin, LOW);// реализуем новое состояние
		} 
		else if (engState == LOW) 
		{ 	  
			engState = HIGH; // выключаем 
			digitalWrite2(stepPin, HIGH); 
		} 
}
};

EncoderH enc[4] = 
{
	EncoderH(pinEncoder1[0],pinEncoder2[1],1), //создание 4х энкодеров и четырех двигателей в соответствии с распиновкой.
	EncoderH(pinEncoder1[0],pinEncoder2[1],2),
	EncoderH(pinEncoder1[0],pinEncoder2[1],3),
	EncoderH(pinEncoder1[0],pinEncoder2[1],4)
};
Engine eng[4] = 
{
	Engine(engPinStep[0],engPinDir[0],engPinEn[0],1),//распиновка, номер двигателя. 1 после номера, если направление обратное.
	Engine(engPinStep[1],engPinDir[1],engPinEn[1],2),
	Engine(engPinStep[2],engPinDir[2],engPinEn[2],3,1),//направление инвертировано
	Engine(engPinStep[3],engPinDir[3],engPinEn[3],4)
};

void setup() 
{
	Timer1.initialize(); 
	Timer1.setPeriod(50);
	Timer1.attachInterrupt( timerIsr ); 
	Timer3.initialize();
	Timer3.setPeriod(1);
	Timer3.attachInterrupt( timerIsr2 );
 
	Serial.begin(9600); // Открываем порт с скоростью передачи в 9600 бод(бит/с)
	for(byte i=0; i<4; i++)
	{
		pinMode2(pinOpen[i], INPUT);
	}
 
	/*//------ Timer1 ----------
	TCCR1A = 0x40;;    // Режим CTC (сброс по совпадению)
	TCCR1B = 0x05;;    // Тактирование от CLK.
			  // Если нужен предделитель :
	// TCCR1B |= (1<<CS11);           // CLK/8
	// TCCR1B |= (1<<CS10)|(1<<CS11); // CLK/64
	// TCCR1B |= (1<<CS12);           // CLK/256
	// TCCR1B |= (1<<CS10)|(1<<CS12); // CLK/1024

	OCR1AH = 0x03;// Верхняя граница счета. Диапазон от 0 до 65535.
			  // Частота прерываний будет = Fclk/(N*(1+OCR1A)) = 1000Гц.
			  // где N - коэф. предделителя (1, 8, 64, 256 или 1024)
	OCR1AL= 0xE8;      
	TIMSK1 = 0x10;   // Разрешить прерывание по совпадению



	//------ Timer2 ----------
	TCCR2A = (1<<WGM21);    // Режим CTC (сброс по совпадению)
	TCCR2B = (1<<CS22);     // Тактирование от CLK.
			  // Если нужен предделитель :
	// TCCR2B = (1<<CS21);                     // CLK/8
	// TCCR2B = (1<<CS20)|(1<<CS21);           // CLK/32
	// TCCR2B = (1<<CS22);                     // CLK/64
	// TCCR2B = (1<<CS20)|(1<<CS22);           // CLK/128
	// TCCR2B = (1<<CS21)|(1<<CS22);           // CLK/256
	// TCCR2B = (1<<CS20)|(1<<CS21)|(1<<CS22); // CLK/1024

	OCR2A = 20;            // Верхняя граница счета. Диапазон от 0 до 255.
			  // Частота прерываний будет = Fclk/(N*(1+OCR2A)) = 20000 Гц.
			  // где N - коэф. предделителя (1, 8, 32, 64, 128, 256 или 1024)
	TIMSK2 = (1<<OCIE2A);   // Разрешить прерывание по совпадению

	sei ();                 // Глобально разрешить прерывания*/
}

void Idn() //далее идут команды. Смотреть ТЗ.
{
	Serial.print("Model: ");
	Serial.print("Arduino MEGA, ");
	Serial.print("ver: ");
	Serial.println("3.20");
}

void StSet(byte ax=0, byte s=0, byte c=0) //дефолтно работает как Clear. 
{
	if(!ax)
	{
		for (byte j=0; j<=3; j++)
		{
			eng[j].SetStat(s);
			eng[j].SetErCode(c);
		}
	}
	else 
	{
		eng[ax-1].SetStat(s);
		eng[ax-1].SetErCode(c);
	}
}

void EnRead(byte p=0) 
{
	if(!p)
	{
		for (byte j=0; j<=2; j++)
		{
			Serial.print(eng[j].power);
			Serial.print(",");
		}
		Serial.println(eng[3].power);
	}
	else Serial.println(eng[p-1].power);
}

void En(byte p=0, boolean ch=0) 
{
	if(!p)
	{
		for (byte j=0; j<=3; j++)
		{
			eng[j].SetPower(ch);
		}
	}
	else eng[p-1].SetPower(ch);;
}

void Stat(byte p=0) //+
{
	if(!p)
	{
		for (byte j=0; j<=2; j++)
		{
			byte stat=eng[j].stat;
			Serial.print(stat);
			Serial.print(",");
		}
		byte stat=eng[3].stat;
		Serial.println(stat);
	}
	else {  byte stat=eng[p-1].stat; Serial.println(stat); }
}

void Err(byte p=0) //+
{
	if(!p)
	{
		for (byte j=0; j<=2; j++)
		{
			byte code=eng[j].erCode;
			Serial.print(code);
			Serial.print(",");
		}
		byte code=eng[3].erCode;
		Serial.println(code);
	}
	else {byte code=eng[p-1].erCode; Serial.println(code); }
}

void Clr(byte p=0) 
{
	if(!p)
	{
		for (byte j=0; j<=3; j++)
		{			
			eng[j].SetErCode(0);
			eng[j].SetStat(0);
		}
	}
	else 
	{
		eng[p-1].SetErCode(0);
		eng[p-1].SetStat(0);
	}
}

void SetError(byte ch, byte p=0)
{
	if(!p)
	{
		for (byte j=0; j<=3; j++)
		{
			eng[j].SetErCode(ch);
		}
	}
	else eng[p-1].SetErCode(ch);
}

void FindTag(byte p) //поиск опорной метки
{
  if(!p)
  {
    for (byte j=0; j<=3; j++)
    {
		forwardDiretion[j]=0;
		eng[j].SetStat(4);
		tagFind[j]=1;
		destination[j]=-999999999;
    }
  }
  else 
  {
    forwardDiretion[p-1]=0;
    eng[p-1].SetStat(4);
    tagFind[p-1]=1;
    destination[p-1]=-999999999;
  }
}
/*void MoveTo(byte p) //движение с выбранной осью. Тут выставляется направление, а само движение идет по таймеру.
{
	long int posit=enc[p-1].pos;
	if(posit>destination[p-1])
	{
		forwardDiretion[p-1]=1;
		eng[p-1].SetStat(4);
		Serial.print("pervochka!");
	}
	else if(posit<destination[p-1])
		{
			forwardDiretion[p-1]=0;
			eng[p-1].SetStat(4);
			Serial.print("dvoechka");
		}
		else {eng[p-1].SetStat(3); Serial.print("paarapauuu!");} 
}*/

void MoveTo(byte p) //движение с выбранной осью. Тут выставляется направление, а само движение идет по таймеру.
{
	long int posit=enc[p-1].pos;
	if(100>destination[p-1])
	{
		forwardDiretion[p-1]=1;
		eng[p-1].SetStat(4);
		Serial.print("pervochka!");
	}
	else if(100<destination[p-1])
		{
			forwardDiretion[p-1]=0;
			eng[p-1].SetStat(4);
			Serial.print("dvoechka");
		}
		else {eng[p-1].SetStat(3); Serial.print("paarapauuu!");}
}




void MoveTo() //аналогично для всех осей
{	
  
	for (byte j=0; j<=3; j++)
	{
		long int posit=enc[j].pos;
		if(posit>destination[j])
		{
			forwardDiretion[j]=0;
			eng[j].SetStat(4);
		}
		else if(posit<destination[j])//!!!!!!!!!!!!
			{
				forwardDiretion[j]=1;
				eng[j].SetStat(4);
			}
			else eng[j].SetStat(3);
		}
}

void Abort(byte p=0)
{
	if(!p)
	{
		for (byte j=0; j<=3; j++)
		{
			eng[j].SetStat(0);
		}
	}
	else eng[p-1].SetStat(0);
}

void Pos(byte p=0)
{
	if(!p)
	{
		for (byte j=0; j<=2; j++)
		{
			int g=enc[j].pos;
			Serial.print(g);
			Serial.print(",");
		}
		int g=enc[3].pos;
		Serial.println(g);
	}
	else {int g=enc[p-1].pos; Serial.println(g);}
}


boolean Command() //тут происходит вызов всех команд. Лучше даже не заглядывать. Пока что(на время отладки) вызовы команд сопровождаются странными сообщениями для однозначного определения происходящего во время выполнения.
{	
	boolean error=true;
	byte param=eng[par[0]].stat;
	Serial.println(par[0]); //ДЕБАААААААААГ!
	Serial.println(par[1]);
	Serial.println(par[2]);
	Serial.println(par[3]);
	Serial.println(par[4]);
	if (param!=1)
	{
		if(strcmp(str,"IDN?")==0)
		{
			error=false;
			Serial.println("versiaya takaya");
			Idn();
		}
		if((strcmp(str,"EN?")==0)||(strcmp(str,"EN*?")==0))
		{
			error=false;
			Serial.println("EN???? tratata");
			EnRead(par[0]);			
		}
		if((strcmp(str,"EN* *")==0))
		{
			error=false;
			Serial.println(par[1]);
			if(par[1]<3) //STATE 1 или 2.
			{
				Serial.println("EN set");
				En(par[0],par[1]);
			}
			else {error=false; Serial.println("STATISTICA"); eng[par[0]].SetErCode(4);} //ошибка 4
		}
		if((strcmp(str,"EN *,*,*,*")==0))
		{
			error=false;
			for(int axis=0; axis<4; axis++)
			{
				if(par[axis+1]<3) //STATE 1 или 2.
				{
					Serial.println("EN set");
					En(axis,par[axis+1]);
				}
				else { Serial.println("STATISTICA"); eng[axis].SetErCode(4);} //ошибка 4
			}
		}
		if((strcmp(str,"STAT?")==0)||(strcmp(str,"STAT*?")==0))
		{
			error=false;
			Serial.println("STAT");
			Stat(par[0]);
		}
		if((strcmp(str,"ERR?")==0)||(strcmp(str,"ERR*?")==0)) //!!!!!!!!!! выводит без вызова
		{
			error=false;
			Serial.println("ERR21");
			Err(par[0]);
		}
		if((strcmp(str,"CLR")==0)||(strcmp(str,"CLR*")==0))
		{
			error=false;
			Serial.println("CLR");
			Clr(par[0]);			
		}
		if((strcmp(str,"MH")==0)||(strcmp(str,"MH*")==0))// двигаться, пока не достигнем опорной метки
		{
			error=false;
			Serial.println("MH");
			FindTag(par[0]);
		}
		if((strcmp(str,"ABORT")==0)||(strcmp(str,"ABORT*")==0))
		{
			error=false;
			Serial.println("ABORT");
			Abort(par[0]);
		}
		if((strcmp(str,"POS?")==0)||(strcmp(str,"POS*?")==0))
		{
			error=false;
			Serial.println("POS");
			Pos(par[0]);
		}
		if((strcmp(str,"MOVE* *")==0))// для одной оси
		{
			error=false;
			destination[par[0]-1]=par[1];
			Serial.println("Move1");
			MoveTo(par[0]);
		}
		if((strcmp(str,"MOVE *,*,*,*")==0)) //для всех осей
		{
			error=false;
			for(byte g=0; g<=3; g++)
			{
				destination[g]=par[g+1];
			}
			Serial.println("Move231");
			MoveTo();
		}
	}
	else
	{
		if((strcmp(str,"CLR")==0)||(strcmp(str,"CLR*")==0))
		{
			error=false;
			Serial.println("CLR");
			Clr(par[0]);			
		}
		if((strcmp(str,"ABORT")==0)||(strcmp(str,"ABORT*")==0))
		{
			error=false;
			Serial.println("ABORT");
			Abort(par[0]);
		}
	}
	return error;
}

void AnalCom() //кажется завершенной. Ошибки применяются ко всем осям, т.к. ось еще не является выбранной. 
{
	int i=0;
	int k=0;
	int nCount=0;
	int nPos=0;
	axis=0;
	byte d=0;
	long int ex=1;
	for (byte j=0; j<=4; j++) //смотреть MOVE в ТЗ (нужно для записи координат в разные оси))
	{
		par[j]=0;
	}
	while(i<inSize) //идем по строке
	{
		if (str[i] == ' ') {d=1; if (d>=5) {StSet(0,1,5);break;}}
		if((str[i] >= '0') && (str[i] <= '9')) //если встретили число
		{
			nCount=0;
			nPos=i;
			while((str[i] >= '0') && (str[i] <= '9'))
			{        
				i++;
				nCount++;
				if(i>=inSize) break;
			}
			for (int j=i-1; j>=nPos; j--)
			{
				par[d]+=(str[j]-'0')*ex;
				ex*=10;
			}
			str[k++]='*';
			Serial.println(str);
			d++;
			if (d>=5) {StSet(0,1,5);break;}
			ex=1;
		}
		str[k++]=str[i++];
		Serial.println(str[i]);
	}
	for(int j=k; j<inSize+1; j++) str[j]='\0';
	Serial.println(str);
	if (par[0]>4) StSet(0,1,5); //ось не найдена(их не больше 4) 
} 

//ISR (TIMER1_COMPB_vect)
void timerIsr()
{
    // Обработчик прерывания таймера 0
	for(volatile byte engNum=0; engNum<=3; engNum++) // пройтись по всем двигателям
	{
		if(destination[engNum]) //смотрим направление движения
		{    //вперед
			if(&enc[engNum].pos>destination)  eng[engNum].SetStat(3);  //если дальше нужного положения, то останавливаем		
		}
		else 
		{    //назад
			if(&enc[engNum].pos<destination)  eng[engNum].SetStat(3);	//если ближе нужного положения, то останавливаем 
		}
		if((eng[engNum].stat)==4)  // если в движении, то едем дальше.
		{
			eng[engNum].Move(forwardDiretion[engNum]); //едем в нужном направлении
		}
		if(tagFind[engNum]) //если идет поиск опорной метки
		{
			if(digitalRead2(pinOpen[engNum])) //при нахождении остановится и обнулить позицию
			{
				eng[engNum].SetStat(2);
				enc[engNum].pos=0;
				tagFind[engNum]=0;
			}
		}
	}

 
};
void timerIsr2()
//ISR (TIMER2_COMPA_vect)
{
	// Обработчик прерывания таймера 1
	for(volatile byte engNum=0; engNum<=3; engNum++) 
	{
		enc[engNum].Update();
	}
}

void loop()
{
  //Serial.println("Sopa!");
	inSize=0; // Сбрасываем переменную
	memset(str, '\0', 256); // Очищаем массив
	if(Serial.available() > 0)
	{
		delay(5); // Ждем, для того, чтобы пришли все данные !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		inSize = Serial.readBytesUntil('\n',str,256);
		//inSize = Serial.available(); // Получаем длину строки и записываем ее в переменную 
		/*for (int i = 0; i < inSize; i++)
		{
		//Serial.print(str[i]);
			str[i] = Serial.read(); // Читаем каждый символ, и пишем его в массив
		}*/
		AnalCom();
		if (Command()) 
		{
			SetError(1); 
			Serial.println("Command not found!");
		}		
	} 
}

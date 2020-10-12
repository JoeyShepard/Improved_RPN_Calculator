#include <stdbool.h>
#include <LPC11xx.h>
#include <LPC_init.h>
#include <PinGPIO.h>

#define DELAY_TIME 48000

#define SPI_READ  0x03
#define SPI_WRITE 0x02

#define BCD_SIGN 0
#define BCD_LEN  1
#define BCD_DEC  2

#define STACK_SIZE 200

#define MATH_CELL_SIZE 120
#define MATH_ENTRY_SIZE 37
#define MATH_LOG_TABLE 114
#define MATH_TRIG_TABLE 113

#define K             "0.60725293500888125616944675250493"
#define log10_factor  "2.30258509299404568401799145468437"
#define pi            "3.1415926535897932384626433832795"
#define pi2           "1.57079632679489661923132169163975"
#define rad_factor    "0.01745329251994329576923690768489"
#define deg_factor    "57.29577951308232087679815481410522"

#define COMP_GT 0
#define COMP_LT 1
#define COMP_EQ 2

#define PROG_COUNT  10
#define PROG_SIZE   4000
#define PROG_HEADER 10  //name(9) + 0
#define PROG_TOTAL  (PROG_SIZE+PROG_HEADER)

/*#define KEY_ENTER     13
#define KEY_BACKSPACE 8
#define KEY_DELETE    83
#define KEY_ESCAPE    27
#define KEY_LEFT      75
#define KEY_RIGHT     77
#define KEY_DOWN      80
#define KEY_UP        72*/

#define SCREEN_WIDTH 18

enum KEYS {KEY_NONE=0,KEY_CLEAR,KEY_SIGN,KEY_DUPE,KEY_ENTER,KEY_SWAP,KEY_FN,
           KEY_PROG,KEY_LEFT,KEY_DOWN,KEY_RIGHT,KEY_2ND,KEY_SQRT,KEY_XRTY,KEY_UP,
           KEY_BACKSPACE,KEY_MOD,KEY_COS,KEY_ACOS,KEY_EX,KEY_10X,KEY_LN,KEY_LOG,KEY_1X,
           KEY_ROUND,KEY_POW,KEY_SIN,KEY_ASIN,KEY_TAN,KEY_ATAN,KEY_SETTINGS,KEY_X2,
           KEY_ESCAPE,KEY_DELETE,KEY_EXEC,KEY_INS};

//KEY_EXEC is a virtual key used to run programs
//* is 42, 9 is 57
//KEY_EXEC is 35
//Could leave hole after '9'
static const char KeyMatrix[]={0 ,KEY_CLEAR,'0','.',KEY_SIGN,'+',
                                  KEY_DUPE,'1','2','3','-',
                                  KEY_SWAP,'4','5','6','*',
                                  KEY_ENTER,'7','8','9','/',
                                  KEY_FN,KEY_PROG,KEY_LEFT,KEY_DOWN,KEY_RIGHT,
                                  KEY_2ND,KEY_SQRT,KEY_XRTY,KEY_UP,KEY_BACKSPACE};

static const char KeyMatrix2nd[]={0,KEY_10X,KEY_ROUND,0,0,0,
                                    KEY_LOG,KEY_SIN,KEY_COS,KEY_TAN,0,
                                    KEY_EX,KEY_X2,KEY_POW,KEY_1X,0,
                                    KEY_LN,KEY_ASIN,KEY_ACOS,KEY_ATAN,KEY_MOD,
                                    0,KEY_SETTINGS,0,0,0,
                                    0,KEY_INS,KEY_XRTY,0,KEY_ESCAPE};

struct SettingsType
{
  int DecPlaces;
  bool DegRad;
  unsigned int LogTableSize;
  unsigned int TrigTableSize;
  bool SciNot;
};

#pragma MM_READ RAM_Read
#pragma MM_WRITE RAM_Write
#pragma MM_ON

static void Calc_Init();

static void LCD_Byte(unsigned char data);
static void LCD_Text(const char *data);
static void LCD_Hex(unsigned char value);
static void LCD_Hex8(unsigned char value);
static void LCD_Init();

static unsigned char SPI_Send(unsigned char value);
static void SPI_Text(const char *data);
static void SPI_Init();

static void Key_Init();

static void SetBlink(bool status);
static void gotoxy(short x, short y);
static unsigned char GetKey();

static void RAM_Write(const unsigned char *a1, const unsigned char byte);
static unsigned char RAM_Read(const unsigned char *a1);

static void MakeTables();
static void SetDecPlaces();
static void ImmedBCD(const char *text, unsigned char *BCD);
static void BufferBCD_EtI(const unsigned char *text, unsigned char *BCD);
static void BufferBCD_ItE(const unsigned char *text, unsigned char *BCD);
static void BufferBCD_ItI(const unsigned char *text, unsigned char *BCD);
static void ImmedBCD_RAM(const char *text, unsigned char *BCD);
static void BufferBCD_ItI(const unsigned char *text, unsigned char *BCD);
static bool IsZero(unsigned char *n1);
static bool IsZero_RAM(unsigned char *n1);
static void AddBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
static void SubBCD(unsigned char *result, const unsigned char *n1, unsigned char *n2);
static void PrintBCD(const unsigned char *BCD, int dec_point);
static void PrintBCD_RAM(const unsigned char *BCD, int dec_point);
static void MultBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
static void DivBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2);
static void ShrinkBCD(unsigned char *dest,unsigned char *src);
static void ShrinkBCD_RAM(unsigned char *dest,unsigned char *src);
static void FullShrinkBCD(unsigned char *n1);
static void FullShrinkBCD_RAM(unsigned char *n1);
static void PadBCD(unsigned char *n1, int amount);
static void PadBCD_RAM(unsigned char *n1, int amount);
static void CopyBCD(unsigned char *dest, unsigned char *src);
static void CopyBCD_EtI(unsigned char *dest, unsigned char *src);
static void CopyBCD_ItE(unsigned char *dest, unsigned char *src);
static void CopyBCD_ItI(unsigned char *dest, unsigned char *src);
static bool LnBCD(unsigned char *result, unsigned char *arg);
static void ExpBCD(unsigned char *result, unsigned char *arg);
static void RolBCD(unsigned char *result, unsigned char *arg, unsigned char amount);
static void RorBCD(unsigned char *result, unsigned char *arg, unsigned char amount);
static void PowBCD(unsigned char *result, unsigned char *base, unsigned char *exp);
static void TanBCD(unsigned char *sine_result,unsigned char *cos_result,unsigned char *arg);
static void AcosBCD(unsigned char *result,unsigned char *arg);
static void AsinBCD(unsigned char *result,unsigned char *arg);
static void AtanBCD(unsigned char *result,unsigned char *arg);
static void CalcTanBCD(unsigned char *result1,unsigned char *result2,unsigned char *result3,unsigned char *arg,unsigned char flag);
static unsigned char CompBCD(const char *num, unsigned char *var);
static unsigned char CompBCD_RAM(const char *num, unsigned char *var);
static unsigned char CompVarBCD(unsigned char *var1, unsigned char *var2);
static unsigned char CompVarBCD_ItE(unsigned char *var1, unsigned char *var2);
static unsigned char CompVarBCD_ItI(unsigned char *var1, unsigned char *var2);
//static unsigned char TrigPrep(unsigned int stack_ptr_copy,unsigned char *cosine);
static unsigned char TrigPrep(unsigned int stack_ptr_copy,int *cosine);

static int ProgLine(int prog, int line, int start, bool fill_buff);
static void ProgLineFill(unsigned char *buffer);
static void ProgLineFillCopy(unsigned char *buffer, const char *msg);

static void DrawStack(bool menu, bool input, int stack_pointer);
static void DrawInput(unsigned char *line, int input_ptr, int offset, bool menu);
static void ErrorMsg(const char *msg);
static void Number2(int num);

static void delay_ms(int ms);

#define ClrLCD() LCD_Byte(0xC)
#define putchar(x) LCD_Byte(x)

#pragma MM_OFFSET 0
#pragma MM_GLOBALS
  //unsigned char p0[260]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD, typing,    CompBCD
  //unsigned char p1[260]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD, DrawStack, CompBCD, CompVarBCD
  //unsigned char p2[260]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD
  //unsigned char p3[260]; //PowBCD, AtanBCD, TrigPrep
  //unsigned char p4[260]; //PowBCD
  //unsigned char p5[260]; //AcosBCD, AsinBCD
  //unsigned char p6[260]; //AcosBCD, AsinBCD
  //unsigned char p7[260]; //AcosBCD, AsinBCD
  //unsigned char buffer[260]; //AddBCD
  //unsigned char perm_buff1[260]; //DivBCD, MultBCD
  //unsigned char perm_buff2[260]; //DivBCD
  //unsigned char perm_buff3[260]; //DivBCD
  //unsigned char logs[MATH_LOG_TABLE*MATH_ENTRY_SIZE];
  unsigned char logs[4218];
  //unsigned char trig[MATH_TRIG_TABLE*MATH_ENTRY_SIZE];
  unsigned char trig[4218];
  //unsigned char perm_zero[4];
  //unsigned char perm_K[36];
  //unsigned char perm_log10[36];
  unsigned char BCD_stack[52000];
  //unsigned char stack_buffer[260];
#pragma MM_END

//Old variables moved from external to internal

unsigned char p0[120]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD, typing,    CompBCD
unsigned char p1[120]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD, DrawStack, CompBCD, CompVarBCD
unsigned char p2[120]; //PowBCD, LnBCD, ExpBCD, TanBCD, AtanBCD
unsigned char p3[120]; //PowBCD, AtanBCD, TrigPrep
unsigned char p4[120]; //PowBCD
unsigned char p5[120]; //AcosBCD, AsinBCD
unsigned char p6[120]; //AcosBCD, AsinBCD
unsigned char p7[120]; //AcosBCD, AsinBCD
unsigned char buffer[120]; //AddBCD
unsigned char perm_buff1[120]; //DivBCD, MultBCD, TanBCD, ExpBCD
unsigned char perm_buff2[120]; //DivBCD, TanBCD, ImmedBCD
unsigned char perm_buff3[120]; //DivBCD

unsigned char perm_zero[4];
unsigned char perm_K[36];
unsigned char perm_log10[36];

unsigned char stack_buffer[120];

//Local buffers for external memory
unsigned char local_buff1[120];
unsigned char local_buff2[120];

struct SettingsType Settings;
unsigned int stack_ptr[2];
unsigned char which_stack;

//PinGPIO LED1(P0_3,GPIO,OUTPUT);

PinGPIO LCD_Busy(P0_4,GPIO,INPUT);
PinGPIO LCD_Clk(P1_0,GPIO,OUTPUT);
PinGPIO LCD_Data(P1_1,GPIO,OUTPUT);
PinGPIO LCD_Reset(P1_2,GPIO,OUTPUT);

PinGPIO SPI_CS(P1_3,GPIO,OUTPUT);
PinGPIO SPI_SCK(P0_10,ALT1,OUTPUT); //0x2
PinGPIO SPI_MISO(P0_8,ALT1,INPUT); //0x1
PinGPIO SPI_MOSI(P0_9,ALT1,OUTPUT); //0x1

PinGPIO KeyIN_A(P0_11,GPIO|PULLUP,INPUT);
PinGPIO KeyIN_B(P0_5,GPIO|PULLUP,INPUT);
PinGPIO KeyIN_C(P0_6,GPIO|PULLUP,INPUT);
PinGPIO KeyIN_D(P1_4,GPIO|PULLUP,INPUT);
PinGPIO KeyIN_E(P1_5,GPIO|PULLUP,INPUT);

PinGPIO KeyOUT_A(P0_7,GPIO|PULLUP,OUTPUT);
PinGPIO KeyOUT_B(P0_3,GPIO|PULLUP,OUTPUT);
PinGPIO KeyOUT_C(P0_2,GPIO|PULLUP,OUTPUT);
PinGPIO KeyOUT_D(P0_1,GPIO|PULLUP,OUTPUT);
PinGPIO KeyOUT_E(P1_9,GPIO|PULLUP,OUTPUT);
PinGPIO KeyOUT_F(P1_8,GPIO|PULLUP,OUTPUT);

int prog_which;

//could store tables in flash if room left
//what to do with extra external ram (3k?)
//Make tables goes past 4218???
//look at clock with logic analyzer

int main(void)
{
  int key,i=0,j=0,k=0,l=0,m=0,x=0,y=0;
  int prog_edit_pos,prog_edit_ptr,prog_edit_offset;
  bool prog_update_cursor,prog_update_line,prog_edit_done;
  int prog_whole_len, prog_seg_start=0, prog_seg_end=0;
  int prog_mov_diff, prog_mov_temp, prog_cursor_offset;
  int next_key, next_key2;
  bool enter_from_delete, prog_insert_line;
  int extra_spaces;
  int prog_counter;
  bool prog_running=false;
  bool prog_title_refresh,prog_title_done;

  bool redraw=true, input=false;
  bool menu=false, redraw_input=false, do_input=false;
  int input_ptr=0, input_offset=0;
  int process_output=0;
  static const char StartInput[]="0123456789.";

  init();

  #pragma MM_ASSIGN_GLOBALS

  LCD_Init();
  Key_Init();
  SPI_Init();
  Calc_Init();

  /*ClrLCD();
  LCD_Text("Key:");
  while(1)
  {
    gotoxy(5,0);
    LCD_Hex(GetKey());
    delay_ms(500);
  }*/

  do
  {
    if (redraw)
    {
      ClrLCD();
      DrawStack(menu,input,stack_ptr[which_stack]);
      redraw=false;
    }
    if (redraw_input)
    {
      DrawInput(p0,input_ptr,input_offset,menu);
      redraw_input=false;
    }

    if (prog_running)
    {
      which_stack=1;
      key=RAM_Read((unsigned char *)(prog_which*PROG_TOTAL+PROG_HEADER+prog_counter));
      which_stack=0;
      if (!key) prog_running=false;
      prog_counter++;
    }
    if (!prog_running)
    {
      key=GetKey();
    }

    j=0;
    do
    {
      if (key==StartInput[j])
      {
        j=0;
        break;
      }
    }while(StartInput[j++]);

    if (j==0)//numbers
    {
      if (input==false)
      {
        if (stack_ptr[which_stack]==STACK_SIZE)
        {
          ErrorMsg("Stack full");
          redraw=true;
        }
        else
        {
          input=true;
          ClrLCD();
          DrawStack(menu,input,stack_ptr[which_stack]);
          input_offset=0;
          input_ptr=1;
          p0[0]=key;
          p0[1]=0;
          redraw_input=true;
          SetBlink(true);
        }
      }
      else
      {
        //if (input_ptr<255)
        if (input_ptr<119)
        {
          k=0;
          for (j=input_ptr;p0[j];j++) k++;
          for (j=k;j>=0;j--) p0[input_ptr+j+1]=p0[input_ptr+j];
          p0[input_ptr++]=key;

          if (input_ptr-input_offset==(SCREEN_WIDTH-1))
          {
            if (p0[input_ptr]==0) input_offset+=0;
            else if ((p0[input_ptr]!=0)&&(p0[input_ptr+1]==0)) input_offset+=0;
            else input_offset++;
          }
          else if (input_ptr-input_offset==(SCREEN_WIDTH))
          {
            if (p0[input_ptr]==0) input_offset++;
            else input_offset++;
          }
          redraw_input=true;
        }
      }
    }
    else//not numbers
    {
      if (input)
      {
        switch(key)
        {
          case KEY_BACKSPACE:
            if (input_ptr)
            {
              input_ptr--;
              if (input_ptr-input_offset==0) input_offset-=2;
              else if (input_ptr-input_offset<2) input_offset--;
              if (input_offset<0) input_offset=0;
              j=input_ptr;
              while (p0[j])
              {
                p0[j]=p0[j+1];
                j++;
              }
              redraw_input=true;
            }
            key=0;
            break;
          case KEY_DELETE:
            j=input_ptr;
            while (p0[j])
            {
              p0[j]=p0[j+1];
              j++;
            }
            redraw_input=true;
            key=0;
            break;
          case KEY_LEFT://left
            if (input_ptr)
            {
              input_ptr--;
              if (input_ptr-input_offset==0) input_offset--;
              if (input_offset<0) input_offset=0;
              redraw_input=true;
            }
            key=0;
            break;
          case KEY_RIGHT://right
            if (p0[input_ptr])
            {
              input_ptr++;
              if (input_ptr-input_offset==(SCREEN_WIDTH-1))
              {
                if (p0[input_ptr]==0) input_offset+=0;
                else if ((p0[input_ptr]!=0)&&(p0[input_ptr+1]==0)) input_offset+=0;
                else input_offset++;
              }
              else if (input_ptr-input_offset==(SCREEN_WIDTH))
              {
                if (p0[input_ptr]==0) input_offset++;
              }
              redraw_input=true;
            }
            key=0;
            break;
          case KEY_ESCAPE:
            SetBlink(false);
            input=false;
            redraw=true;
            key=0;
            break;
          case KEY_ENTER:
            do_input=true;
            key=0;
            break;
          case KEY_CLEAR:
            input_ptr=0;
            input_offset=0;
            p0[0]=0;
            redraw_input=true;
            key=0;
            break;
          case KEY_DOWN:
          case KEY_UP:
            key=0;
            break;
          default: //other key pressed during input
            do_input=true;
        }
      }

      //else redraw=true; //not inputting and not number

      if (do_input)
      {
        x=0;
        for (j=0;p0[j];j++)
        {
          if (p0[j]=='.') x++;
          if (x==2) break;
        }
        if (x==2)
        {
          ErrorMsg("Invalid input");
          redraw_input=true;
        }
        else if (p0[0]==0)
        {
          SetBlink(false);
          input=false;
          redraw=true;
          key=0;
        }
        else
        {
          SetBlink(false);
          BufferBCD_ItE(p0,BCD_stack+stack_ptr[which_stack]*MATH_CELL_SIZE);

          if (IsZero(BCD_stack+stack_ptr[which_stack]*MATH_CELL_SIZE)&&(BCD_stack[stack_ptr[which_stack]*MATH_CELL_SIZE+BCD_SIGN])) BCD_stack[stack_ptr[which_stack]*MATH_CELL_SIZE+BCD_SIGN]=0;
          FullShrinkBCD(BCD_stack+stack_ptr[which_stack]*MATH_CELL_SIZE);
          stack_ptr[which_stack]++;
          input=false;
        }
        redraw=true;
        do_input=false;
      }
      process_output=0;
      switch (key)
      {
        case '+':
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
            CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            //AddBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            AddBCD(stack_buffer,p0,p1);
            process_output=2;
            redraw=true;
          }
          break;
        case '-':
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
            CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            //SubBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            SubBCD(stack_buffer,p0,p1);
            process_output=2;
            redraw=true;
          }
          break;
        case '/':
          if (stack_ptr[which_stack]>=2)
          {
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
            {
              ErrorMsg("Divide by zero");
            }
            else
            {
              CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
              CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              //DivBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              DivBCD(stack_buffer,p0,p1);
              process_output=2;
            }
            redraw=true;
          }
          break;
        case KEY_MOD:
          if (stack_ptr[which_stack]>=2)
          {
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
            {
              ErrorMsg("Invalid Input");
            }
            else
            {
              CopyBCD_EtI(p3,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
              CopyBCD_EtI(p2,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);

              if (p3[BCD_SIGN]==1) j=1;
              else j=0;
              p3[BCD_SIGN]=0;
              p2[BCD_SIGN]=0;

              while(1)
              {
                SubBCD(p0,p3,p2);
                if (p0[BCD_SIGN])
                {
                  CopyBCD_ItI(stack_buffer,p3);
                  break;
                }
                else CopyBCD_ItI(p3,p0);
              }
              stack_buffer[BCD_SIGN]=j;
              process_output=2;
            }
            redraw=true;
          }
          break;
        case '*':
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
            CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            //MultBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            MultBCD(stack_buffer,p0,p1);
            process_output=2;
            redraw=true;
          }
          break;
        case KEY_BACKSPACE:
        case KEY_DELETE:
          if (stack_ptr[which_stack]>=1)
          {
            stack_ptr[which_stack]--;
            redraw=true;
          }
          break;
        case KEY_ENTER:
        case KEY_DUPE://dupe
          if (stack_ptr[which_stack]>=1)
          {
            if (stack_ptr[which_stack]==STACK_SIZE)
            {
              ErrorMsg("Stack full");
            }
            else
            {
              CopyBCD(BCD_stack+(stack_ptr[which_stack])*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              stack_ptr[which_stack]++;
            }
            redraw=true;
          }
          break;
        case KEY_LEFT:
          if (stack_ptr[which_stack]>=1)
          {
            CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            RolBCD(stack_buffer,local_buff1,1);
            process_output=1;
            redraw=true;
          }
          break;
        case KEY_RIGHT:
          if (stack_ptr[which_stack]>=1)
          {
            CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            //RorBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,1);
            RorBCD(stack_buffer,p0,1);
            process_output=1;
            redraw=true;
          }
          break;
        case KEY_UP:
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(stack_buffer,BCD_stack);
            for (i=0;i<((int)(stack_ptr[which_stack]-1));i++)
            {
              CopyBCD(BCD_stack+i*MATH_CELL_SIZE,BCD_stack+(i+1)*MATH_CELL_SIZE);
            }
            CopyBCD_ItE(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,stack_buffer);
            redraw=true;
          }
          break;
        case KEY_DOWN:
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            for (i=(stack_ptr[which_stack]-1);i>0;i--)
            {
              CopyBCD(BCD_stack+(i)*MATH_CELL_SIZE,BCD_stack+(i-1)*MATH_CELL_SIZE);
            }
            CopyBCD_ItE(BCD_stack,stack_buffer);
            redraw=true;
          }
          break;
        /*case 'b'://test
          P1OUT&=~RAM_BANK;
          TestRAM();
          P1OUT|=RAM_BANK;
          TestRAM();

          if (which_stack==0) P1OUT&=~RAM_BANK;
          else P1OUT|=RAM_BANK;

          ClrLCD();
          LCD_Text("Press a key...");
          gotoxy(0,2);
          LCD_Text("Key: 00");
          gotoxy(0,3);
          LCD_Text("Code: 00");
          i=0;
          j=0;
          do
          {
            j=GetKey();
            if (j!=i)
            {
              gotoxy(5,2);
              LCD_Hex(j);
              gotoxy(6,3);
              LCD_Hex(KeyMatrix[j]);
              i=j;
            }
          //} while (KeyMatrix[j]!=27);
          } while(1);
          break;*/
        case KEY_EXEC:
          prog_counter=0;
          prog_running=true;
          key=0;
          redraw=true;
          break;
        case KEY_COS://cosine
          if (stack_ptr[which_stack]>=1)
          {
            BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=0;
            TrigPrep(stack_ptr[which_stack],&j);
            if (IsZero_RAM(p3)) ImmedBCD("1",stack_buffer);
            else TanBCD(p4,stack_buffer,p3);
            if (j==1) stack_buffer[BCD_SIGN]=1;
            process_output=1;
            redraw=true;
          }
          break;
        case KEY_ACOS:
          if (stack_ptr[which_stack]>=1)
          {
            process_output=1;
            i=CompBCD("0",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            j=CompBCD("1",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            k=CompBCD("-1",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            if (i==COMP_EQ) ImmedBCD_RAM("90",stack_buffer);
            else if (j==COMP_EQ) ImmedBCD_RAM("0",stack_buffer);
            else if (k==COMP_EQ) ImmedBCD_RAM("180",stack_buffer);
            else if ((j==COMP_LT)||(k==COMP_GT))
            {
              ErrorMsg("Invalid input");
              process_output=0;
            }
            else
            {
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              AcosBCD(stack_buffer,local_buff1);
            }
            redraw=true;
          }
          break;
        case KEY_EX:
          if (stack_ptr[which_stack]>=1)
          {
            ImmedBCD_RAM("177",p0);
            CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            SubBCD(p2,p1,p0);
            if (p2[BCD_SIGN]==0)
            {
              ErrorMsg("Argument\ntoo large");
            }
            else
            {
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              ExpBCD(stack_buffer,local_buff1);
              process_output=1;
            }
            redraw=true;
          }
          break;
        case KEY_10X:
          if (stack_ptr[which_stack]>=1)
          {
            x=0;
            //j=CompVarBCD(perm_zero,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)) j=COMP_EQ;
            else if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==1) j=COMP_GT;
            else j=COMP_LT;

            if (j==COMP_EQ) ImmedBCD_RAM("1",stack_buffer);
            else
            {
              if (j==COMP_GT) j=1;
              else j=0;
              BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=0;

              CopyBCD_EtI(p2,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              CopyBCD_ItI(p1,p2);
              p2[BCD_LEN]=p2[BCD_DEC];
              SubBCD(p0,p1,p2);

              //if (CompVarBCD(p2,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)==COMP_EQ)//x is an integer
              if (IsZero_RAM(p0))//x is an integer
              {
                ImmedBCD_RAM("117",p2);
                SubBCD(p0,p1,p2);
                //if (CompVarBCD(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,p2)==COMP_GT)
                if (p0[BCD_SIGN]==0)
                {
                  i=255;
                }
                else
                {
                  i=BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+3]*100;
                  i+=BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+4]*10;
                  i+=BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+5];
                  if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_DEC]==2) i/=10;
                  else if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_DEC]==1) i/=100;
                }

                if (i>254)
                {
                  ErrorMsg("Invalid input");
                  BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=j;
                  x=1;
                }
                else
                {
                  stack_buffer[BCD_SIGN]=0;
                  stack_buffer[BCD_DEC]=i+1;
                  stack_buffer[BCD_LEN]=i+1;
                  stack_buffer[3]=1;
                  for (k=0;k<i;k++)
                  {
                    stack_buffer[k+4]=0;
                  }
                }
              }
              else
              {
                ImmedBCD_RAM("10",p5);
                CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
                PowBCD(stack_buffer,p5,local_buff1);
                if (stack_buffer[BCD_DEC]>(Settings.DecPlaces)) stack_buffer[BCD_LEN]=stack_buffer[BCD_DEC];
                else if (stack_buffer[BCD_LEN]>(Settings.DecPlaces))
                {
                  stack_buffer[BCD_LEN]=Settings.DecPlaces;
                }
              }

              if ((j)&&(x==0))
              {
                ImmedBCD_RAM("1",p2);
                DivBCD(p3,p2,stack_buffer);
                CopyBCD_ItI(stack_buffer,p3);
              }
            }
            if (x==0) process_output=1;
            redraw=true;
          }
          break;
        case KEY_PROG:
          //reusing redraw here
          redraw=true;
          x=0;
          y=0;
          do
          {
            if (redraw)
            {
              ClrLCD();
              which_stack=1;
              for (i=y;i<(y+4);i++)
              {
                gotoxy(0,i-y);
                LCD_Text(" ");
                LCD_Hex8(i+1);
                LCD_Text(": ");
                for (j=0;j<10;j++)
                {
                  k=RAM_Read((unsigned char *)(PROG_TOTAL*i+j));
                  if (!k) break;
                  if (k>=0x20) LCD_Byte(k);
                }
                //j=RAM_Read((unsigned char *)(PROG_TOTAL*i+0));
                //j+=(RAM_Read((unsigned char *)(PROG_TOTAL*i+1))<<8);
                //gotoxy(SCREEN_WIDTH-3,i-y);
                //LCD_Hex8(j>>8);
                //LCD_Hex(j&0xFF);
                for (j=0;RAM_Read((unsigned char *)(i*PROG_TOTAL+PROG_HEADER+j));j++);
                gotoxy(SCREEN_WIDTH-4,i-y);
                j=j%0x1000; //Max program size is 2^12
                LCD_Byte('0'+j/1000);
                j=j%1000;
                LCD_Byte('0'+j/100);
                j=j%100;
                LCD_Byte('0'+j/10);
                j=j%10;
                LCD_Byte('0'+j);
              }
              which_stack=0;

              gotoxy(0,x-y);
              LCD_Text(">");
              redraw=false;
            }
            key=GetKey();
            if ((key==KEY_DOWN)&&(x<8))
            {
              x++;
              if ((x-y)>3)
              {
                y++;
                redraw=true;
              }
              else
              {
                gotoxy(0,(x-y-1));
                LCD_Text(" ");
                gotoxy(0,(x-y));
                LCD_Text(">");
              }
            }
            else if ((key==KEY_UP)&&(x>0))
            {
              x--;
              if (x<y)
              {
                y--;
                redraw=true;
              }
              else
              {
                gotoxy(0,(x-y+1));
                LCD_Text(" ");
                gotoxy(0,(x-y));
                LCD_Text(">");
              }
            }
            else if (key==KEY_ENTER)
            {
              key=0;
              prog_title_refresh=true;
              which_stack=1;
              for (j=0;j<10;j++)
              {
                k=RAM_Read((unsigned char *)(PROG_TOTAL*x+j));
                p0[j]=k;
                if (!k) break;
              }
              which_stack=0;
              prog_edit_pos=j;


              SetBlink(true);
              while((key!=KEY_ESCAPE)&&(key!=KEY_ENTER))
              {
                if (prog_title_refresh)
                {
                  gotoxy(4,(x-y));
                  prog_title_done=false;
                  for (j=0;j<10;j++)
                  {
                    if (!p0[j]) prog_title_done=true;
                    if (prog_title_done) LCD_Text(" ");
                    else (LCD_Byte(p0[j]));
                  }
                  gotoxy(prog_edit_pos+4,(x-y));
                }
                prog_title_refresh=true;

                key=GetKey();
                if (key==KEY_BACKSPACE)
                {
                  if (prog_edit_pos)
                  {
                    prog_edit_pos--;
                    p0[prog_edit_pos]=0;
                  }
                }
                else if ((key>='0')&&(key<='9'))
                {
                  if (prog_edit_pos==0)
                  {
                    p0[0]=key;
                    prog_edit_pos++;
                    p0[1]=0;
                  }
                  else
                  {
                    p0[prog_edit_pos-1]++;
                  }
                }
                else if (key==KEY_RIGHT)
                {
                  p0[prog_edit_pos]='a';
                  prog_edit_pos++;
                  p0[prog_edit_pos]=0;
                }
                else prog_title_refresh=false;
              }

              j=0;
              k=0;
              prog_edit_pos=0;
              redraw=true;
              prog_update_cursor=true;
              prog_update_line=true;
              SetBlink(true);
              key=0;
              next_key=0;
              next_key2=0;
              prog_cursor_offset=0;
              enter_from_delete=false;

              which_stack=1;
              for (prog_whole_len=0;RAM_Read((unsigned char *)(x*PROG_TOTAL+PROG_HEADER+prog_whole_len));prog_whole_len++);
              which_stack=0;

              do
              {
                if (((key>='0')&&(key<='9'))||(key=='.'))
                {
                  if ((((p0[prog_edit_pos-1]>='0')&&(p0[prog_edit_pos-1]<='9'))||(p0[prog_edit_pos-1]=='.'))||(key=='.')||(p0[0]==0)||(p0[0]==KEY_INS))
                  {
                    p0[prog_edit_pos]=key;
                    prog_edit_pos++;
                    p0[prog_edit_pos]=0;
                    prog_update_cursor=true;
                    prog_update_line=false;
                  }
                }
                else if ((key=='+')||
                         (key=='-')||
                         (key=='*')||
                         (key=='/')||
                         (key==KEY_DUPE)||
                         (key==KEY_SWAP)||
                         (key==KEY_SQRT)||
                         (key==KEY_XRTY)||
                         (key==KEY_MOD)||
                         (key==KEY_COS)||
                         (key==KEY_ACOS)||
                         (key==KEY_EX)||
                         (key==KEY_10X)||
                         (key==KEY_LN)||
                         (key==KEY_LOG)||
                         (key==KEY_1X)||
                         (key==KEY_ROUND)||
                         (key==KEY_POW)||
                         (key==KEY_SIN)||
                         (key==KEY_ASIN)||
                         (key==KEY_TAN)||
                         (key==KEY_ATAN)||
                         (key==KEY_X2)||
                         (key==KEY_SIGN))

                {
                  p0[prog_edit_pos]=key;
                  prog_edit_pos++;
                  p0[prog_edit_pos]=0;
                  prog_update_cursor=true;
                  prog_update_line=false;
                  next_key=KEY_ENTER;
                  if (prog_edit_pos!=1) next_key2=KEY_DOWN;
                }
                else if (key==KEY_INS)
                {
                  p0[prog_edit_pos]=key;
                  prog_edit_pos++;
                  p0[prog_edit_pos]=0;
                  prog_update_cursor=true;
                  prog_update_line=false;
                  next_key=KEY_ENTER;
                }
                else if ((key==KEY_DOWN)||((prog_cursor_offset>0)&&(key==0)))
                {
                  if (j<0xFFF)
                  {
                    if (!((p0[0]==KEY_INS)&&(prog_edit_pos>0)))
                    {
                      ProgLine(x,j,0,true);
                      //Redraws line before going down. discards changes
                      prog_edit_done=false;

                      SetBlink(false);
                      extra_spaces=0;
                      gotoxy(4,j-k);

                      prog_edit_done=false;
                      for (l=0;l<(SCREEN_WIDTH-4);l++)
                      {
                        if (p1[l]==0) prog_edit_done=true;
                        if (prog_edit_done) LCD_Byte(' ');
                        else if (p1[l]!=KEY_INS) LCD_Byte(p1[l]);
                      }
                      if (prog_edit_done==false)
                      {
                        gotoxy(SCREEN_WIDTH-1,j-k);
                        LCD_Byte('>');
                      }
                      SetBlink(true);

                      prog_mov_temp=ProgLine(x,j,0,false);
                      which_stack=1;
                      if (RAM_Read((unsigned char *)prog_mov_temp)) j++;
                      which_stack=0;
                      if ((j-k)>3)
                      {
                        k++;
                        redraw=true;
                      }
                      prog_update_cursor=true;
                    }
                    else
                    {
                      p0[0]=0;
                      prog_edit_pos=0;
                      next_key=KEY_ENTER;
                      prog_cursor_offset=-1;
                      enter_from_delete=true;
                    }
                  }
                  if ((prog_cursor_offset>0)&&(key==0)) prog_cursor_offset--;
                }
                else if ((key==KEY_UP)||((prog_cursor_offset<0)&&(key==0)))
                {
                  if (j>0)
                  {
                    if (!((p0[0]==KEY_INS)&&(prog_edit_pos>0)))
                    {
                      ProgLine(x,j,0,true);
                      //Redraws line before going down. discards changes
                      prog_edit_done=false;

                      SetBlink(false);
                      extra_spaces=0;
                      gotoxy(4,j-k);
                      prog_edit_done=false;
                      for (l=0;l<(SCREEN_WIDTH-4);l++)
                      {
                        if (p1[l]==0) prog_edit_done=true;
                        if (prog_edit_done) LCD_Byte(' ');
                        else if (p1[l]!=KEY_INS) LCD_Byte(p1[l]);
                      }
                      if (prog_edit_done==false)
                      {
                        gotoxy(SCREEN_WIDTH-1,j-k);
                        LCD_Byte('>');
                      }
                      SetBlink(true);

                      j--;
                      if (j<k)
                      {
                        k--;
                        redraw=true;
                      }
                      prog_update_cursor=true;
                    }
                    else
                    {
                      p0[0]=0;
                      prog_edit_pos=0;
                      next_key=KEY_ENTER;
                      //next_key2=KEY_UP;
                      prog_cursor_offset=-2;
                      enter_from_delete=true;
                    }
                  }
                  if ((prog_cursor_offset<0)&&(key==0)) prog_cursor_offset++;
                }
                else if (key==KEY_BACKSPACE)
                {
                  //printf("*%d ",prog_edit_pos);
                  if ((p0[0]==KEY_INS)&&(prog_edit_pos==1))
                  {
                    prog_edit_pos=0;
                  }

                  if (prog_edit_pos>0)
                  {
                    prog_edit_pos--;
                    p0[prog_edit_pos]=0;
                    prog_update_cursor=true;
                    prog_update_line=false;
                  }
                  else
                  {
                    enter_from_delete=true;
                    next_key=KEY_ENTER;
                    prog_cursor_offset=-2;
                  }
                }
                else if (key==KEY_ENTER)
                {
                  if (!enter_from_delete)
                  {
                    if (prog_edit_pos==0)
                    {
                      p0[0]=KEY_ENTER;
                      prog_edit_pos=1;
                      p0[1]=0;
                    }
                  }
                  else enter_from_delete=false;

                  if ((p0[0]==KEY_INS)&&(prog_edit_pos>0))
                  {
                    if (prog_edit_pos==1)
                    {
                      p0[0]=KEY_ENTER;
                    }
                    else
                    {
                      for (i=1;i<prog_edit_pos;i++)
                      {
                        p0[i-1]=p0[i];
                      }
                      prog_edit_pos--;
                      p0[prog_edit_pos]=0;
                    }
                  }

                  if (prog_seg_end<=prog_seg_start)
                  {
                    //typing on the last line. nothing to do
                    //printf("*Just copy*");
                  }
                  else
                  {
                    //printf("\n*Orig:%d*\n*New:%d*",prog_seg_end-prog_seg_start,prog_edit_pos);

                    if ((prog_seg_end-prog_seg_start)>prog_edit_pos)
                    {
                      prog_mov_diff=(prog_seg_end-prog_seg_start)-prog_edit_pos;
                      //printf("\nshrink. Start:%d",prog_seg_start+prog_edit_pos);
                      which_stack=1;
                      for (i=prog_seg_start+prog_edit_pos;RAM_Read((unsigned char *)i);i++)
                      {
                        RAM_Write((unsigned char *)(i),RAM_Read((unsigned char *)(i+prog_mov_diff)));
                      }
                      which_stack=0;
                      //for (i=0;i<prog_edit_pos;i++) RAM_Write((unsigned char *)(prog_seg_start+i),p0[i]);
                    }
                    else if ((prog_seg_end-prog_seg_start)<prog_edit_pos)
                    {
                      prog_mov_diff=prog_edit_pos-(prog_seg_end-prog_seg_start);
                      //printf("\nexpand");
                      which_stack=1;
                      for (i=prog_seg_end;RAM_Read((unsigned char *)i);i++)
                      {
                        //printf("%d ",i-prog_seg_end);
                      }
                      //printf("Final i:%d*\n",i);
                      for (;i>=prog_seg_end;i--)
                      {
                        //printf("
                        RAM_Write((unsigned char *)(i+prog_mov_diff),RAM_Read((unsigned char *)(i)));
                      }
                      which_stack=0;
                    }
                  }

                  for (i=0;i<prog_edit_pos;i++)
                  {
                    prog_mov_temp=p0[i];
                    which_stack=1;
                    RAM_Write((unsigned char *)(prog_seg_start+i),prog_mov_temp);
                    which_stack=0;
                  }
                  redraw=true;
                  //prog_update_cursor=true;
                  if (prog_cursor_offset<0) prog_cursor_offset++;
                  else next_key=KEY_DOWN;
                }

                if (redraw)
                {
                  SetBlink(false);
                  ClrLCD();
                  for (i=k;i<(k+4);i++)
                  {
                    gotoxy(0,i-k);
                    LCD_Hex8((i>>8)&0xF);
                    LCD_Hex(i&0xFF);
                    LCD_Text(":");
                    ProgLine(x,i,0,true);
                    prog_edit_done=false;
                    for (l=0;l<(SCREEN_WIDTH-4);l++)
                    {
                      if (p1[l]==0) prog_edit_done=true;
                      if (prog_edit_done) LCD_Byte(' ');
                      else if (p1[l]!=KEY_INS) LCD_Byte(p1[l]);
                    }
                    if (prog_edit_done==false)
                    {
                      gotoxy(SCREEN_WIDTH-1,i-k);
                      LCD_Byte('>');
                    }
                  }
                  redraw=false;
                  prog_update_cursor=true;
                  //redundant?
                  prog_update_line=true;
                }

                if (prog_update_cursor)
                {
                  if (prog_update_line)
                  {
                    prog_seg_end=ProgLine(x,j+1,0,false);
                    prog_seg_start=ProgLine(x,j,0,true);
                    //ProgLine(x,j,0,true);
                    for (prog_edit_pos=0;p0[prog_edit_pos];prog_edit_pos++);
                  }
                  else
                  {
                    ProgLineFill(p1);
                  }
                  //if (!prog_update_line) printf("*false*");
                  prog_update_line=true;
                  prog_edit_done=false;

                  SetBlink(false);

                  extra_spaces=0;
                  gotoxy(4,j-k);
                  for (m=0;p1[m];m++);
                  if (m<(SCREEN_WIDTH-4))
                  {
                    m=0;
                    //4 characters for address and colon
                    for (i=0;i<(SCREEN_WIDTH-4);i++)
                    {
                      if (p1[i]!=KEY_INS)
                      {
                        if (p1[i]==0) prog_edit_done=true;
                        if (prog_edit_done) LCD_Byte(' ');
                        else LCD_Byte(p1[i]);
                      }
                      else extra_spaces++;
                    }
                    for (i=0;p1[i];i++);
                  }
                  else
                  {
                    //1 for <?
                    m=m-(SCREEN_WIDTH-4)+1;
                    LCD_Byte('<');
                    //4 characters for address and colon
                    for (i=1;i<(SCREEN_WIDTH-4);i++)
                    {
                      if (p1[i+m]!=KEY_INS)
                      {
                        if (p1[i+m]==0) prog_edit_done=true;
                        if (prog_edit_done) LCD_Byte(' ');
                        else  LCD_Byte(p1[i+m]);
                      }
                      else extra_spaces++;
                    }
                    i=SCREEN_WIDTH-5;
                  }
                  gotoxy(i+4-extra_spaces,j-k);

                  prog_update_cursor=false;
                  SetBlink(true);
                }
                if (next_key)
                {
                  key=next_key;
                  next_key=0;
                }
                else if (next_key2)
                {
                  key=next_key2;
                  next_key2=0;
                }
                else if (prog_cursor_offset!=0)
                {
                  key=0;
                }
                else key=GetKey();
              }while (key!=KEY_ESCAPE);
              key=0;
              redraw=true;
            }
            SetBlink(false);
          }while (key!=KEY_ESCAPE);
          key=0;
          redraw=true;
          break;
        /*case 'i'://pi
          if (stack_ptr[which_stack]==STACK_SIZE)
          {
            ErrorMsg("Stack full");
          }
          else
          {
            stack_ptr[which_stack]++;
            ImmedBCD(pi,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_LEN]=1+Settings.DecPlaces;
          }
          redraw=true;
          break;*/
        case KEY_LN:
          if (stack_ptr[which_stack]>=1)
          {
            //CopyBCD_EtI(p1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            //SubBCD(p2,perm_zero_RAM,p0);
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)||
                (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==1))
            {
              ErrorMsg("Invalid input");
            }
            else
            {
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              //if (LnBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)) process_output=1;
              if (LnBCD(stack_buffer,local_buff1)) process_output=1;
              else ErrorMsg("Invalid input");
            }
            redraw=true;
          }
          break;
        case KEY_LOG:
          if (stack_ptr[which_stack]>=1)
          {
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)||
                (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==1))
            {
              ErrorMsg("Invalid input");
            }
            else
            {
              x=0;
              if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+3]==1)
              {
                CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);

                j=p0[BCD_DEC];
                k=p0[BCD_LEN];
                for (i=k;i>j;i--)
                {
                  if (p0[i+2]==0) k--;
                  else break;
                }

                p0[BCD_LEN]=k;

                if (p0[BCD_LEN]==p0[BCD_DEC])
                {
                  p0[3]=0;
                  if (IsZero_RAM(p0))
                  {
                    stack_buffer[BCD_LEN]=3;
                    stack_buffer[BCD_DEC]=3;
                    stack_buffer[BCD_SIGN]=0;
                    i=p0[BCD_LEN]-1;
                    stack_buffer[3]=i/100;
                    stack_buffer[4]=(i%100)/10;
                    stack_buffer[5]=(i%10);
                    FullShrinkBCD_RAM(stack_buffer);
                    process_output=1;
                    x=1;
                  }
                }
              }

              if (!x)
              {
                CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
                if (LnBCD(p3,local_buff1))
                {
                  DivBCD(stack_buffer,p3,perm_log10);
                  process_output=1;
                }
                else ErrorMsg("Argument\ntoo large");
              }
            }
            redraw=true;
          }
          break;
        case KEY_SIGN:// +/-
          if (stack_ptr[which_stack]>=1)
          {
            //if (CompVarBCD(perm_zero,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE)!=COMP_EQ)
            if (!IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
            {
              if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==0)
              {
                BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=1;
              }
              else BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=0;
            }
            redraw=true;
          }
          break;
        case KEY_1X:// 1/x
          if (stack_ptr[which_stack]>=1)
          {
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
            {
              ErrorMsg("Divide by zero");
            }
            else
            {
              ImmedBCD_RAM("1",p0);
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              DivBCD(stack_buffer,p0,local_buff1);
              process_output=1;
            }
            redraw=true;
          }
          break;
        case KEY_ROUND://round
          if (stack_ptr[which_stack]>=1)
          {
            if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_LEN]>BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_DEC])
            {
              BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_LEN]=BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_DEC];
              if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_LEN]+3]>4)
              {
                ImmedBCD_RAM("1",p0);
                CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
                AddBCD(stack_buffer,p0,local_buff1);
              }
              else CopyBCD_EtI(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              process_output=1;
            }
            redraw=true;
          }
          break;
        case KEY_POW://'p'://y^x
        case KEY_XRTY://'r'://x root y
          if (stack_ptr[which_stack]>=2)
          {
            x=0;
            if (key==KEY_XRTY)
            {
              if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
              {
                ErrorMsg("Invalid Input");
                x=1;
              }
              else
              {
                ImmedBCD_RAM("1",p3);
                CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
                DivBCD(p5,p3,local_buff1);
              }
            }
            else CopyBCD_EtI(p5,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);

            if (x==0)
            {
              j=CompVarBCD_ItI(perm_zero,p5);
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
              k=CompVarBCD_ItI(perm_zero,local_buff1);

              if (k==COMP_GT)
              {
                y=1;
                BCD_stack[(stack_ptr[which_stack]-2)*MATH_CELL_SIZE+BCD_SIGN]=0;
              }
              else y=0;

              if (j==COMP_GT)
              {
                y+=2;
                p5[BCD_SIGN]=0;
              }

              if (k==COMP_EQ) ImmedBCD_RAM("0",stack_buffer);
              else if (j==COMP_EQ) ImmedBCD_RAM("1",stack_buffer);
              else
              {
                CopyBCD_EtI(p2,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
                p2[BCD_LEN]=p2[BCD_DEC];
                CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
                if (CompVarBCD_ItI(p2,local_buff1)==COMP_EQ) j=1;
                else j=0;
                CopyBCD_ItI(p2,p5);
                p2[BCD_LEN]=p2[BCD_DEC];
                if (CompVarBCD_ItI(p2,p5)==COMP_EQ) j+=2;

                if (y&1)//y is negative
                {
                  if (j&2)//x is an integer
                  {
                    ///BCD_stack[(stack_ptr[which_stack]-2)*MATH_CELL_SIZE+BCD_SIGN]=0;
                  }
                  else
                  {
                    ErrorMsg("Invalid input");
                    BCD_stack[(stack_ptr[which_stack]-2)*MATH_CELL_SIZE+BCD_SIGN]=(y&1);
                    x=1;
                  }
                }

                if (x==0)
                {
                  CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
                  PowBCD(stack_buffer,local_buff1,p5);
                  if (stack_buffer[BCD_DEC]>(Settings.DecPlaces))
                  {
                    stack_buffer[BCD_LEN]=stack_buffer[BCD_DEC];
                  }
                  else if (stack_buffer[BCD_LEN]>(Settings.DecPlaces))
                  {
                    stack_buffer[BCD_LEN]=Settings.DecPlaces;
                  }

                  if ((j==3)&&(true)) //both are integers
                  {
                    //this works only if slightly above
                    //stack_buffer[BCD_LEN]=stack_buffer[BCD_DEC];
                    //could also fail if exp is less than 1
                  }

                  if (y&2)
                  {
                    ImmedBCD_RAM("1",p2);
                    DivBCD(p3,p2,stack_buffer);
                    CopyBCD_ItI(stack_buffer,p3);
                  }

                  if (y&1)
                  {
                    if (p5[p5[BCD_DEC]+2]%2==1)
                    {
                      stack_buffer[BCD_SIGN]=(y&1);
                    }
                  }
                }
                if (x==0) process_output=2;
              }
            }
            redraw=true;
          }
          break;
        case KEY_SQRT://sqrt
          if (stack_ptr[which_stack]>=1)
          {
            if (IsZero(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE))
            {
              ImmedBCD_RAM("0",stack_buffer);
              process_output=1;
            }
            else if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==1)
            {
              ErrorMsg("Invalid input");
            }
            else
            {
              ImmedBCD_RAM("0.5",p5);
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              //PowBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,p5);
              PowBCD(stack_buffer,local_buff1,p5);
              if (stack_buffer[BCD_DEC]>(Settings.DecPlaces)) stack_buffer[BCD_LEN]=stack_buffer[BCD_DEC];
              else if (stack_buffer[BCD_LEN]>(Settings.DecPlaces))
              {
                stack_buffer[BCD_LEN]=Settings.DecPlaces;
              }
              process_output=1;
            }
            redraw=true;
          }
          break;
        case KEY_SIN:
        case KEY_TAN:
          if (stack_ptr[which_stack]>=1)
          {
            if (BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]==1)
            {
              BCD_stack[(stack_ptr[which_stack]-1)*MATH_CELL_SIZE+BCD_SIGN]=0;
              j=1;
            }
            else j=0;
            j+=TrigPrep(stack_ptr[which_stack],&k);

            if ((key==KEY_TAN)&&(CompBCD_RAM("90",p3)==COMP_EQ))
            {
              ErrorMsg("Invalid input");
            }
            else
            {
              TanBCD(stack_buffer,p4,p3);

              if (CompBCD_RAM("90",p3)==COMP_EQ) ImmedBCD_RAM("1",stack_buffer);
              if (j==1) stack_buffer[BCD_SIGN]=1;
              if (k==1) p4[BCD_SIGN]=1;

              if (key==KEY_TAN)
              {
                DivBCD(p3,stack_buffer,p4);
                CopyBCD_ItI(stack_buffer,p3);
              }
              process_output=1;
            }
            redraw=true;
          }
          break;
        case KEY_ASIN:
          if (stack_ptr[which_stack]>=1)
          {
            process_output=1;
            i=CompBCD("0",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            j=CompBCD("1",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            k=CompBCD("-1",BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            if (i==COMP_EQ) ImmedBCD_RAM("0",stack_buffer);
            else if (j==COMP_EQ) ImmedBCD_RAM("90",stack_buffer);
            else if (k==COMP_EQ) ImmedBCD_RAM("-90",stack_buffer);
            else if ((j==COMP_LT)||(k==COMP_GT))
            {
              ErrorMsg("Invalid input");
              process_output=0;
            }
            else
            {
              CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
              AsinBCD(stack_buffer,local_buff1);
            }
            redraw=true;
          }
          break;
        case KEY_ATAN:
          if (stack_ptr[which_stack]>=1)
          {
            CopyBCD_EtI(local_buff1,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            AtanBCD(stack_buffer,local_buff1);
            process_output=1;
            redraw=true;
          }
          break;
        case KEY_SETTINGS:
          ClrLCD();
          gotoxy(0,0);
          LCD_Text(">Accuracy:");
          gotoxy(1,1);
          LCD_Text("Deg/rad:");
          gotoxy(1,2);
          LCD_Text("Sci. Not.");
          //gotoxy(1,3);
          //LCD_Text("Battery:");

          i=Settings.DecPlaces;
          j=39;
          x=5;
          y=0;
          key=0;
          do
          {
            if (x!=5)
            {
              //if (KeyMatrix[key]) delay_ms(500);
              //key=KeyMatrix[GetKey()];
              //delay_ms(500);
              key=GetKey();
            }
            j++;
            if (j==40)
            {
              j=0;
              /*gotoxy(13,3);

              ADC10CTL0 = SREF_1 + REFON + REF2_5V + ADC10ON + ADC10SHT_3;
              ADC10CTL1 = INCH_11;
              delay_ms(1);
              ADC10CTL0 |= ENC + ADC10SC;

              while (!(ADC10CTL0 & ADC10IFG));
              unsigned long temp=(ADC10MEM * 50001)/10241;

              putchar('0'+temp/1000);
              LCD_Num(temp);
              LCD_Text("mV");

              ADC10CTL0&=~ENC;
              ADC10CTL0=0;*/
            }

            if ((key==KEY_DOWN)||(key==KEY_UP))
            {
              gotoxy(0,y);
              putchar(' ');

              if ((key==KEY_DOWN)&&(y<2)) y++;
              else if ((key==KEY_UP)&&(y>0)) y--;

              gotoxy(0,y);
              putchar('>');
            }
            else if (key==KEY_RIGHT)
            {
              if (y==0)
              {
                if (Settings.DecPlaces<32)
                {
                  Settings.DecPlaces++;
                  x=1;
                }
              }
            }
            else if (key==KEY_LEFT)
            {
              if (y==0)
              {
                if (Settings.DecPlaces>6)
                {
                  Settings.DecPlaces--;
                  x=1;
                }
              }
            }

            if ((key==KEY_LEFT)||(key==KEY_RIGHT))
            {
              if (y==1)
              {
                Settings.DegRad=!Settings.DegRad;
                x=2;
              }
              else if (y==2)
              {
                Settings.SciNot=!Settings.SciNot;
                x=3;
              }
            }

            if ((x==1)||(x==5))
            {
              gotoxy(SCREEN_WIDTH-3,0);
              Number2(Settings.DecPlaces);
            }
            if ((x==2)||(x==5))
            {
              gotoxy(SCREEN_WIDTH-4,1);
              if (Settings.DegRad) LCD_Text("Deg");
              else LCD_Text("Rad");
            }
            if ((x==3)||(x==5))
            {
              gotoxy(SCREEN_WIDTH-4,2);
              if (Settings.SciNot) LCD_Text(" On");
              else LCD_Text("Off");
            }

            x=0;
          } while ((key!=KEY_ESCAPE)&&(key!=KEY_ENTER));

          if (i!=Settings.DecPlaces) SetDecPlaces();

          key=0;
          redraw=true;
          break;
        case KEY_SWAP:
          if (stack_ptr[which_stack]>=2)
          {
            CopyBCD_EtI(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            CopyBCD(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE);
            CopyBCD_ItE(BCD_stack+(stack_ptr[which_stack]-2)*MATH_CELL_SIZE,stack_buffer);
            redraw=true;
          }
          break;
        case KEY_X2://x^2
          if (stack_ptr[which_stack]>=1)
          {
            CopyBCD_EtI(p0,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
            CopyBCD_ItI(local_buff1,p0);
            MultBCD(stack_buffer,p0,local_buff1);
            process_output=1;
            redraw=true;
          }
          break;
        case KEY_CLEAR://clear
          if (stack_ptr[which_stack]>=1)
          {
            stack_ptr[which_stack]=0;
            redraw=true;
          }
          break;
        default:
          process_output=0;
      }

      if (Settings.DegRad==false)
      {
        if ((key==KEY_ATAN)||(key==KEY_ACOS)||(key==KEY_ASIN))
        {
          if (process_output>0)
          {
            CopyBCD(p0,stack_buffer);
            ImmedBCD(deg_factor,p1);
            DivBCD(stack_buffer,p0,p1);
          }
        }
      }

      if (process_output==2) stack_ptr[which_stack]--;
      if (process_output>0)
      {
        FullShrinkBCD_RAM(stack_buffer);
        if (IsZero_RAM(stack_buffer)&&(stack_buffer[BCD_SIGN])) stack_buffer[BCD_SIGN]=0;
        CopyBCD_ItE(BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE,stack_buffer);
      }
    }
  //} while (key!=KEY_ESCAPE);
  }while(1);

  return 0;
}

static void Calc_Init()
{
  int i;
  Settings.DecPlaces=24;
  Settings.DegRad=true;
  Settings.LogTableSize=MATH_LOG_TABLE;
  Settings.TrigTableSize=MATH_TRIG_TABLE;

  stack_ptr[0]=0;
  stack_ptr[1]=0;
  which_stack=0;

  LCD_Text("Writing RAM..");
  ImmedBCD_RAM("0",perm_zero);
  ImmedBCD_RAM(K,perm_K);
  ImmedBCD_RAM(log10_factor,perm_log10);
  LCD_Text("Done\r\n");

  LCD_Text("Writing RAM0..");
  MakeTables();
  SetDecPlaces();
  LCD_Text("Done\r\n");

  //Use second page for programs
  LCD_Text("Writing RAM1..");
  which_stack=1;
  for (i=0;i<PROG_COUNT*PROG_TOTAL;i++)
  {
    RAM_Write((const unsigned char*)(i),0);
  }

  RAM_Write((const unsigned char*)(2*PROG_TOTAL+0),'T');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+1),'e');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+2),'s');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+3),'t');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+4),0);

  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+0),'1');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+1),'2');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+2),'3');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+3),KEY_ENTER);
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+4),'4');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+5),'5');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+6),'6');
  RAM_Write((const unsigned char*)(2*PROG_TOTAL+PROG_HEADER+7),'+');
  which_stack=0;
  LCD_Text("Done");
  //delay_ms(500);

  /*
  static unsigned const char CustomChars[]={0,0,0,0,0,0,7,4,         //CUST_NW
                                            0,0,0,0,0,0,28,4,        //CUST_NE
                                            4,4,4,4,4,4,4,4,         //CUST_WE
                                            4,7,0,0,0,0,0,0,         //CUST_SW
                                            0,31,0,0,0,0,0,0,        //CUST_S
                                            31,16,23,21,23,16,31,0,  //CUST_OK_O
                                            31,1,21,25,21,1,31,0,    //CUST_OK_K
                                            31,0,17,27,27,17,0,31};  //Busy hour glass

  LCD_Byte(0x40,0);
  for (i=0;i<64;i++) LCD_Byte(CustomChars[i],1);*/
}

static void LCD_Byte(unsigned char data)
{
  int i;
  //LED1=1;
  for (i=0;i<8;i++)
  {
    LCD_Clk=0;
    //delay_ms(1);
    if (data&(1<<i)) LCD_Data=1;
    else LCD_Data=0;
    //delay_ms(1);
    LCD_Clk=1;
    //delay_ms(1);
  }
  while (LCD_Busy.read());
  //LED1=0;
}

static void LCD_Text(const char *data)
{
  unsigned long i;
  for (i=0;data[i];i++) LCD_Byte(data[i]);
}

static void LCD_Hex(unsigned char value)
{
  if ((value>>4)>9) LCD_Byte((value>>4)-10+'A');
  else LCD_Byte((value>>4)+'0');
  if ((value&0xF)>9) LCD_Byte((value&0xF)-10+'A');
  else LCD_Byte((value&0xF)+'0');
}

static void LCD_Hex8(unsigned char value)
{
  if ((value&0xF)>9) LCD_Byte((value&0xF)-10+'A');
  else LCD_Byte((value&0xF)+'0');
}

static void LCD_Init()
{
  LCD_Busy.config();
  LCD_Clk.config();
  LCD_Data.config();
  LCD_Reset.config();
  //LED1=1;
  LCD_Clk=1;
  LCD_Reset=1;
  delay_ms(10);
  LCD_Reset=0;
  delay_ms(10);
  LCD_Reset=1;
  while (LCD_Busy.read());
  //LED1=0;
}

static void SetBlink(bool status)
{
  LCD_Byte(0x1F);
  LCD_Byte(0x43);
  if (status) LCD_Byte(1);
  else LCD_Byte(0);
}

static void gotoxy(short x, short y)
{
  LCD_Byte(0x1F);
  LCD_Byte(0x24);
  LCD_Byte(x*7);
  LCD_Byte(0);
  LCD_Byte(y);
  LCD_Byte(0);
}

static unsigned char GetKey()
{
  bool shift=false;
  bool Fn=false;
  volatile unsigned char retval=0,key=0;
  int i,j;
  while (1)
  {
    for (j=0;j<2;j++)
    {
      if (j==0)
      {
        shift=false;
        Fn=false;
      }
      KeyOUT_A=1;
      KeyOUT_B=1;
      KeyOUT_C=1;
      KeyOUT_D=1;
      KeyOUT_E=1;
      KeyOUT_F=1;
      retval=0;
      for (i=0;i<6;i++)
      {
        switch (i)
        {
          case 0:
            KeyOUT_A=0;
            break;
          case 1:
            KeyOUT_A=1;
            KeyOUT_B=0;
            break;
          case 2:
            KeyOUT_B=1;
            KeyOUT_C=0;
            break;
          case 3:
            KeyOUT_C=1;
            KeyOUT_D=0;
            break;
          case 4:
            KeyOUT_D=1;
            KeyOUT_E=0;
            break;
          case 5:
            KeyOUT_E=1;
            KeyOUT_F=0;
            break;
        }
        //delay_ms(10);
        if (!KeyIN_A.read()) retval=1;
        if (!KeyIN_B.read()) retval=2;
        if (!KeyIN_C.read()) retval=3;
        if (!KeyIN_D.read()) retval=4;
        if (!KeyIN_E.read()) retval=5;
        if (retval)
        {
          if (j==0)
          {
            if ((KeyMatrix[i*5+retval]==KEY_2ND)&&(shift==false))
            {
              delay_ms(10);
              shift=true;
            }
            if ((KeyMatrix[i*5+retval]==KEY_FN)&&(Fn==false))
            {
              delay_ms(10);
              Fn=true;
            }
            retval=0;
          }
          else if (key==0)
          {
            if (KeyMatrix[i*5+retval]==KEY_2ND) retval=0;
            else if (KeyMatrix[i*5+retval]==KEY_FN) retval=0;
            else
            {
              if (shift)
              {
                delay_ms(10);//debounce
                //LCD_Text("&");
                //LCD_Hex(i*5+retval);
                //LCD_Text("&");
                //delay_ms(1000);
                key=KeyMatrix2nd[i*5+retval];
              }
              else if (Fn)
              {
                delay_ms(10);
                key=KeyMatrix[i*5+retval];
                if ((key>='1')&&(key<='9'))
                {
                  prog_which=key-'1';
                  key=KEY_EXEC;
                  //LCD_Text("*Fn:Yes*");
                }
                else
                {
                  key=0;
                  //LCD_Text("*Fn:No*");
                }
                //delay_ms(1000);
              }
              else
              {
                delay_ms(10);//debounce
                //LCD_Text("<");
                //LCD_Hex(i*5+retval);
                //LCD_Text(">");
                //delay_ms(1000);
                key=KeyMatrix[i*5+retval];
              }
            }
          }
        }
      }
    }
    if ((key)&&(retval==0)) return key;
  }
  return retval;
}

static unsigned char GetKeyEmu()
{
  static int counter=0;
  const char msg[]={'9',KEY_ENTER,'q',0};

  /*const char msg[]={
                    '7','6',KEY_ENTER,
                    'f',KEY_ENTER,
                    '1','2','3','+',
                    '4','5','*',
                    //KEY_ENTER,
                    KEY_ESCAPE,KEY_ESCAPE,'b',
                    0};*/
  delay_ms(500);
  if (!msg[counter]) return 0;
  return (unsigned char)msg[counter++];
}

unsigned char SPI_Send(unsigned char value)
{
  while ((LPC_SSP0->SR & (BIT4))!=0);
  LPC_SSP0->DR=value;
  while ((LPC_SSP0->SR & (BIT4))!=0);
  return LPC_SSP0->DR;
}

void SPI_Text(const char *data)
{
  unsigned int i;
  for (i=0;data[i];i++) SPI_Send(data[i]);
}

static void SPI_Init()
{
  int i;
  unsigned char Dummy;

  SPI_CS.config();

  SPI_CS=1;

  LPC_SYSCON->PRESETCTRL |= BIT0;     //Table 9 - Reset SPI0
  LPC_SYSCON->SYSAHBCLKCTRL |= BIT11;  //Table 21 - Enable clock
  //SEEMS TO WORK WITH SRAM BUT MIGHT NOT!
  LPC_SYSCON->SSP0CLKDIV = 2;    //Table 22 - Clock divider. 0x0 is shut off.

  SPI_MISO.config();
  SPI_MOSI.config();

  //Has to be in following order!
  LPC_IOCON->SCK_LOC&=~0x3;
  LPC_IOCON->SCK_LOC|=0x0; //Select P0_10 as SPI clock

  SPI_SCK.config();

  LPC_SSP0->CR0=0x7|(0<<4)|(0<<6)|(0<<7)|0;//8 bit, SPI, CPOL=0, CPHA=0, no divider
  LPC_SSP0->CPSR=2; //Table 212 - Minimum divider
  LPC_SSP0->CR1=0|BIT1|(0<<2)|(0<<3);//no loop back, SPI Enable, master, no slave disable

  //how about (void)LPC_SSP0->DR;
  for (i=0;i<8;i++) Dummy=LPC_SSP0->DR;
  (void)Dummy;
}

static void Key_Init()
{
  KeyIN_A.config();
  KeyIN_B.config();
  KeyIN_C.config();
  KeyIN_D.config();
  KeyIN_E.config();

  KeyOUT_A.config();
  KeyOUT_B.config();
  KeyOUT_C.config();
  KeyOUT_D.config();
  KeyOUT_E.config();
  KeyOUT_F.config();

  KeyOUT_A=1;
  KeyOUT_B=1;
  KeyOUT_C=1;
  KeyOUT_D=1;
  KeyOUT_E=1;
  KeyOUT_F=1;
}

static unsigned char RAM_Read(const unsigned char *a1)
{
  unsigned char retval;
  SPI_CS=0;
  SPI_Send(SPI_READ);
  SPI_Send(which_stack);
  SPI_Send(((unsigned int)a1)>>8);
  SPI_Send(((unsigned int)a1)&0xFF);
  retval=SPI_Send(0);
  SPI_CS=1;
  return retval;
}

static void RAM_Write(const unsigned char *a1, const unsigned char byte)
{
  SPI_CS=0;
  SPI_Send(SPI_WRITE);
  SPI_Send(which_stack);
  SPI_Send(((unsigned int)a1)>>8);
  SPI_Send(((unsigned int)a1)&0xFF);
  SPI_Send(byte);
  SPI_CS=1;
}

//maybe BCD isn't that efficient
static void MakeTables()
{
  //Log table
  static const unsigned char table[]={
  17 ,0x88,0x72,0x28,0x39,0x11,0x16,0x72,0x99,0x96,0x05,0x40,0x57,0x11,0x54,0x66,0x46,0x60,
  17 ,0x44,0x36,0x14,0x19,0x55,0x58,0x36,0x49,0x98,0x02,0x70,0x28,0x55,0x77,0x33,0x23,0x30,
  17 ,0x22,0x18,0x07,0x09,0x77,0x79,0x18,0x24,0x99,0x01,0x35,0x14,0x27,0x88,0x66,0x61,0x65,
  17 ,0x11,0x09,0x03,0x54,0x88,0x89,0x59,0x12,0x49,0x50,0x67,0x57,0x13,0x94,0x33,0x30,0x83,
  17 ,0x05,0x54,0x51,0x77,0x44,0x44,0x79,0x56,0x24,0x75,0x33,0x78,0x56,0x97,0x16,0x65,0x41,
  17 ,0x02,0x77,0x25,0x88,0x72,0x22,0x39,0x78,0x12,0x37,0x66,0x89,0x28,0x48,0x58,0x32,0x71,
  17 ,0x01,0x38,0x62,0x94,0x36,0x11,0x19,0x89,0x06,0x18,0x83,0x44,0x64,0x24,0x29,0x16,0x35,
  16 ,0x69,0x31,0x47,0x18,0x05,0x59,0x94,0x53,0x09,0x41,0x72,0x32,0x12,0x14,0x58,0x18,
  16 ,0x40,0x54,0x65,0x10,0x81,0x08,0x16,0x43,0x81,0x97,0x80,0x13,0x11,0x54,0x64,0x35,
  16 ,0x22,0x31,0x43,0x55,0x13,0x14,0x20,0x97,0x55,0x76,0x62,0x95,0x09,0x03,0x09,0x83,
  16 ,0x11,0x77,0x83,0x03,0x56,0x56,0x38,0x34,0x54,0x53,0x87,0x94,0x10,0x94,0x70,0x52,
  16 ,0x06,0x06,0x24,0x62,0x18,0x16,0x43,0x48,0x42,0x58,0x06,0x06,0x13,0x20,0x40,0x42,
  16 ,0x03,0x07,0x71,0x65,0x86,0x66,0x75,0x36,0x88,0x37,0x10,0x28,0x20,0x75,0x96,0x77,
  16 ,0x01,0x55,0x04,0x18,0x65,0x35,0x96,0x52,0x54,0x15,0x08,0x54,0x04,0x60,0x42,0x45,
  15 ,0x77,0x82,0x14,0x04,0x42,0x05,0x49,0x48,0x94,0x74,0x62,0x90,0x00,0x61,0x14,
  15 ,0x38,0x98,0x64,0x04,0x15,0x65,0x73,0x23,0x01,0x39,0x37,0x34,0x30,0x95,0x84,
  15 ,0x19,0x51,0x22,0x01,0x31,0x26,0x17,0x49,0x43,0x96,0x74,0x04,0x95,0x31,0x84,
  15 ,0x09,0x76,0x08,0x59,0x73,0x05,0x54,0x58,0x89,0x59,0x60,0x82,0x49,0x08,0x02,
  15 ,0x04,0x88,0x16,0x20,0x79,0x50,0x13,0x51,0x18,0x85,0x37,0x04,0x96,0x92,0x65,
  15 ,0x02,0x44,0x11,0x08,0x27,0x52,0x73,0x62,0x70,0x91,0x60,0x47,0x90,0x85,0x82,
  15 ,0x01,0x22,0x06,0x28,0x62,0x52,0x56,0x77,0x37,0x16,0x23,0x05,0x53,0x67,0x16,
  14 ,0x61,0x03,0x32,0x93,0x68,0x06,0x38,0x52,0x49,0x13,0x15,0x87,0x89,0x65,
  14 ,0x30,0x51,0x71,0x12,0x47,0x31,0x86,0x37,0x85,0x69,0x06,0x95,0x14,0x17,
  14 ,0x15,0x25,0x86,0x72,0x64,0x83,0x62,0x39,0x74,0x05,0x75,0x73,0x25,0x13,
  14 ,0x07,0x62,0x93,0x65,0x42,0x75,0x67,0x57,0x21,0x55,0x88,0x52,0x96,0x85,
  14 ,0x03,0x81,0x46,0x89,0x98,0x96,0x85,0x88,0x94,0x80,0x71,0x17,0x84,0x98,
  14 ,0x01,0x90,0x73,0x46,0x81,0x38,0x25,0x40,0x94,0x15,0x46,0x94,0x42,0x51,
  13 ,0x95,0x36,0x73,0x86,0x16,0x59,0x18,0x82,0x33,0x90,0x84,0x15,0x51,
  13 ,0x47,0x68,0x37,0x04,0x45,0x16,0x32,0x34,0x18,0x44,0x34,0x61,0x75,
  13 ,0x23,0x84,0x18,0x55,0x06,0x79,0x85,0x75,0x87,0x10,0x42,0x36,0x79,
  13 ,0x11,0x92,0x09,0x28,0x24,0x45,0x35,0x44,0x57,0x08,0x75,0x79,0x16,
  13 ,0x05,0x96,0x04,0x64,0x29,0x99,0x03,0x38,0x56,0x18,0x58,0x25,0x32,
  13 ,0x02,0x98,0x02,0x32,0x19,0x43,0x60,0x61,0x11,0x47,0x31,0x97,0x05,
  13 ,0x01,0x49,0x01,0x16,0x10,0x82,0x82,0x53,0x54,0x89,0x03,0x91,0x82,
  12 ,0x74,0x50,0x58,0x05,0x69,0x16,0x82,0x52,0x64,0x72,0x34,0x52,
  12 ,0x37,0x25,0x29,0x02,0x91,0x52,0x30,0x20,0x17,0x58,0x25,0x70,
  12 ,0x18,0x62,0x64,0x51,0x47,0x49,0x62,0x33,0x55,0x74,0x27,0x31,
  12 ,0x09,0x31,0x32,0x25,0x74,0x18,0x17,0x97,0x64,0x69,0x00,0x06,
  12 ,0x04,0x65,0x66,0x12,0x87,0x19,0x93,0x19,0x04,0x05,0x97,0x61,
  12 ,0x02,0x32,0x83,0x06,0x43,0x62,0x67,0x64,0x57,0x45,0x98,0x32,
  12 ,0x01,0x16,0x41,0x53,0x21,0x82,0x01,0x58,0x55,0x08,0x75,0x62,
  11 ,0x58,0x20,0x76,0x60,0x91,0x17,0x73,0x34,0x13,0x32,0x12,
  11 ,0x29,0x10,0x38,0x30,0x45,0x63,0x10,0x18,0x71,0x39,0x66,
  11 ,0x14,0x55,0x19,0x15,0x22,0x82,0x60,0x97,0x26,0x88,0x23,
  11 ,0x07,0x27,0x59,0x57,0x61,0x41,0x56,0x95,0x61,0x23,0x72,
  11 ,0x03,0x63,0x79,0x78,0x80,0x70,0x85,0x09,0x55,0x06,0x76,
  11 ,0x01,0x81,0x89,0x89,0x40,0x35,0x44,0x20,0x21,0x14,0x60,
  10 ,0x90,0x94,0x94,0x70,0x17,0x72,0x51,0x46,0x47,0x61,
  10 ,0x45,0x47,0x47,0x35,0x08,0x86,0x36,0x07,0x21,0x38,
  10 ,0x22,0x73,0x73,0x67,0x54,0x43,0x20,0x62,0x10,0x08,
  10 ,0x11,0x36,0x86,0x83,0x77,0x21,0x60,0x95,0x67,0x39,
  10 ,0x05,0x68,0x43,0x41,0x88,0x60,0x80,0x63,0x99,0x28,
  10 ,0x02,0x84,0x21,0x70,0x94,0x30,0x40,0x36,0x03,0x54,
  10 ,0x01,0x42,0x10,0x85,0x47,0x15,0x20,0x19,0x02,0x74,
  9 ,0x71,0x05,0x42,0x73,0x57,0x60,0x09,0x76,0x61,
  9 ,0x35,0x52,0x71,0x36,0x78,0x80,0x04,0x94,0x62,
  9 ,0x17,0x76,0x35,0x68,0x39,0x40,0x02,0x48,0x89,
  9 ,0x08,0x88,0x17,0x84,0x19,0x70,0x01,0x24,0x84,
  9 ,0x04,0x44,0x08,0x92,0x09,0x85,0x00,0x62,0x52,
  9 ,0x02,0x22,0x04,0x46,0x04,0x92,0x50,0x31,0x28,
  9 ,0x01,0x11,0x02,0x23,0x02,0x46,0x25,0x15,0x65,
  8 ,0x55,0x51,0x11,0x51,0x23,0x12,0x57,0x83,
  8 ,0x27,0x75,0x55,0x75,0x61,0x56,0x28,0x91,
  8 ,0x13,0x87,0x77,0x87,0x80,0x78,0x14,0x46,
  8 ,0x06,0x93,0x88,0x93,0x90,0x39,0x07,0x23,
  8 ,0x03,0x46,0x94,0x46,0x95,0x19,0x53,0x61,
  8 ,0x01,0x73,0x47,0x23,0x47,0x59,0x76,0x81,
  7 ,0x86,0x73,0x61,0x73,0x79,0x88,0x40,
  7 ,0x43,0x36,0x80,0x86,0x89,0x94,0x20,
  7 ,0x21,0x68,0x40,0x43,0x44,0x97,0x10,
  7 ,0x10,0x84,0x20,0x21,0x72,0x48,0x55,
  7 ,0x05,0x42,0x10,0x10,0x86,0x24,0x27,
  7 ,0x02,0x71,0x05,0x05,0x43,0x12,0x14,
  7 ,0x01,0x35,0x52,0x52,0x71,0x56,0x07,
  6 ,0x67,0x76,0x26,0x35,0x78,0x03,
  6 ,0x33,0x88,0x13,0x17,0x89,0x02,
  6 ,0x16,0x94,0x06,0x58,0x94,0x51,
  6 ,0x08,0x47,0x03,0x29,0x47,0x25,
  6 ,0x04,0x23,0x51,0x64,0x73,0x63,
  6 ,0x02,0x11,0x75,0x82,0x36,0x81,
  6 ,0x01,0x05,0x87,0x91,0x18,0x41,
  5 ,0x52,0x93,0x95,0x59,0x20,
  5 ,0x26,0x46,0x97,0x79,0x60,
  5 ,0x13,0x23,0x48,0x89,0x80,
  5 ,0x06,0x61,0x74,0x44,0x90,
  5 ,0x03,0x30,0x87,0x22,0x45,
  5 ,0x01,0x65,0x43,0x61,0x22,
  4 ,0x82,0x71,0x80,0x61,
  4 ,0x41,0x35,0x90,0x31,
  4 ,0x20,0x67,0x95,0x15,
  4 ,0x10,0x33,0x97,0x58,
  4 ,0x05,0x16,0x98,0x79,
  4 ,0x02,0x58,0x49,0x39,
  4 ,0x01,0x29,0x24,0x70,
  3 ,0x64,0x62,0x35,
  3 ,0x32,0x31,0x17,
  3 ,0x16,0x15,0x59,
  3 ,0x08,0x07,0x79,
  3 ,0x04,0x03,0x90,
  3 ,0x02,0x01,0x95,
  3 ,0x01,0x00,0x97,
  2 ,0x50,0x49,
  2 ,0x25,0x24,
  2 ,0x12,0x62,
  2 ,0x06,0x31,
  2 ,0x03,0x15,
  2 ,0x01,0x58,
  1 ,0x79,
  1 ,0x39,
  1 ,0x20,
  1 ,0x10,
  1 ,0x05,
  1 ,0x02,
  1 ,0x01,
  //Trig table
  //this could be done by doubling
  17 ,0x45,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  17 ,0x26,0x56,0x50,0x51,0x17,0x70,0x77,0x98,0x93,0x51,0x57,0x21,0x93,0x72,0x04,0x53,0x29,
  17 ,0x14,0x03,0x62,0x43,0x46,0x79,0x26,0x47,0x85,0x82,0x89,0x23,0x20,0x15,0x91,0x63,0x42,
  17 ,0x07,0x12,0x50,0x16,0x34,0x89,0x01,0x79,0x75,0x61,0x95,0x33,0x00,0x84,0x12,0x06,0x84,
  17 ,0x03,0x57,0x63,0x34,0x37,0x49,0x97,0x35,0x10,0x30,0x68,0x47,0x78,0x91,0x44,0x58,0x82,
  17 ,0x01,0x78,0x99,0x10,0x60,0x82,0x46,0x06,0x93,0x07,0x15,0x02,0x49,0x77,0x60,0x79,0x09,
  16 ,0x89,0x51,0x73,0x71,0x02,0x11,0x07,0x43,0x13,0x64,0x12,0x16,0x82,0x30,0x79,0x53,
  16 ,0x44,0x76,0x14,0x17,0x08,0x60,0x55,0x30,0x73,0x09,0x43,0x53,0x82,0x54,0x23,0x82,
  16 ,0x22,0x38,0x10,0x50,0x03,0x68,0x53,0x80,0x75,0x12,0x35,0x33,0x54,0x24,0x30,0x59,
  16 ,0x11,0x19,0x05,0x67,0x70,0x66,0x20,0x68,0x87,0x27,0x54,0x75,0x79,0x70,0x34,0x72,
  16 ,0x05,0x59,0x52,0x89,0x18,0x93,0x80,0x36,0x68,0x17,0x44,0x24,0x13,0x44,0x04,0x23,
  16 ,0x02,0x79,0x76,0x45,0x26,0x17,0x00,0x36,0x74,0x59,0x91,0x79,0x11,0x92,0x36,0x83,
  16 ,0x01,0x39,0x88,0x22,0x71,0x42,0x26,0x50,0x14,0x62,0x86,0x87,0x63,0x57,0x24,0x36,
  15 ,0x69,0x94,0x11,0x36,0x75,0x35,0x29,0x18,0x45,0x75,0x24,0x89,0x32,0x87,0x82,
  15 ,0x34,0x97,0x05,0x68,0x50,0x70,0x40,0x11,0x05,0x84,0x42,0x77,0x35,0x40,0x77,
  15 ,0x17,0x48,0x52,0x84,0x26,0x98,0x04,0x49,0x52,0x15,0x80,0x88,0x73,0x44,0x18,
  15 ,0x08,0x74,0x26,0x42,0x13,0x69,0x37,0x80,0x26,0x02,0x61,0x92,0x68,0x64,0x27,
  15 ,0x04,0x37,0x13,0x21,0x06,0x87,0x23,0x34,0x56,0x75,0x78,0x22,0x83,0x81,0x83,
  15 ,0x02,0x18,0x56,0x60,0x53,0x43,0x93,0x47,0x83,0x84,0x70,0x43,0x88,0x58,0x09,
  15 ,0x01,0x09,0x28,0x30,0x26,0x72,0x00,0x71,0x48,0x85,0x70,0x39,0x80,0x29,0x58,
  14 ,0x54,0x64,0x15,0x13,0x36,0x00,0x85,0x44,0x04,0x52,0x09,0x67,0x46,0x64,
  14 ,0x27,0x32,0x07,0x56,0x68,0x00,0x48,0x93,0x22,0x46,0x91,0x06,0x02,0x51,
  14 ,0x13,0x66,0x03,0x78,0x34,0x00,0x25,0x24,0x26,0x26,0x06,0x30,0x80,0x30,
  14 ,0x06,0x83,0x01,0x89,0x17,0x00,0x12,0x71,0x83,0x75,0x85,0x75,0x12,0x55,
  14 ,0x03,0x41,0x50,0x94,0x58,0x50,0x06,0x37,0x13,0x20,0x78,0x20,0x02,0x82,
  14 ,0x01,0x70,0x75,0x47,0x29,0x25,0x03,0x18,0x71,0x76,0x99,0x76,0x57,0x23,
  13 ,0x85,0x37,0x73,0x64,0x62,0x51,0x59,0x37,0x78,0x07,0x46,0x60,0x59,
  13 ,0x42,0x68,0x86,0x82,0x31,0x25,0x79,0x69,0x12,0x73,0x43,0x09,0x29,
  13 ,0x21,0x34,0x43,0x41,0x15,0x62,0x89,0x84,0x59,0x32,0x92,0x77,0x02,
  13 ,0x10,0x67,0x21,0x70,0x57,0x81,0x44,0x92,0x30,0x03,0x49,0x03,0x81,
  13 ,0x05,0x33,0x60,0x85,0x28,0x90,0x72,0x46,0x15,0x06,0x37,0x35,0x07,
  13 ,0x02,0x66,0x80,0x42,0x64,0x45,0x36,0x23,0x07,0x53,0x76,0x52,0x93,
  13 ,0x01,0x33,0x40,0x21,0x32,0x22,0x68,0x11,0x53,0x76,0x95,0x49,0x64,
  12 ,0x66,0x70,0x10,0x66,0x11,0x34,0x05,0x76,0x88,0x48,0x65,0x22,
  12 ,0x33,0x35,0x05,0x33,0x05,0x67,0x02,0x88,0x44,0x24,0x43,0x91,
  12 ,0x16,0x67,0x52,0x66,0x52,0x83,0x51,0x44,0x22,0x12,0x23,0x37,
  12 ,0x08,0x33,0x76,0x33,0x26,0x41,0x75,0x72,0x11,0x06,0x11,0x86,
  12 ,0x04,0x16,0x88,0x16,0x63,0x20,0x87,0x86,0x05,0x53,0x05,0x95,
  12 ,0x02,0x08,0x44,0x08,0x31,0x60,0x43,0x93,0x02,0x76,0x52,0x98,
  12 ,0x01,0x04,0x22,0x04,0x15,0x80,0x21,0x96,0x51,0x38,0x26,0x49,
  11 ,0x52,0x11,0x02,0x07,0x90,0x10,0x98,0x25,0x69,0x13,0x24,
  11 ,0x26,0x05,0x51,0x03,0x95,0x05,0x49,0x12,0x84,0x56,0x62,
  11 ,0x13,0x02,0x75,0x51,0x97,0x52,0x74,0x56,0x42,0x28,0x31,
  11 ,0x06,0x51,0x37,0x75,0x98,0x76,0x37,0x28,0x21,0x14,0x16,
  11 ,0x03,0x25,0x68,0x87,0x99,0x38,0x18,0x64,0x10,0x57,0x08,
  11 ,0x01,0x62,0x84,0x43,0x99,0x69,0x09,0x32,0x05,0x28,0x54,
  10 ,0x81,0x42,0x21,0x99,0x84,0x54,0x66,0x02,0x64,0x27,
  10 ,0x40,0x71,0x10,0x99,0x92,0x27,0x33,0x01,0x32,0x13,
  10 ,0x20,0x35,0x55,0x49,0x96,0x13,0x66,0x50,0x66,0x07,
  10 ,0x10,0x17,0x77,0x74,0x98,0x06,0x83,0x25,0x33,0x03,
  10 ,0x05,0x08,0x88,0x87,0x49,0x03,0x41,0x62,0x66,0x52,
  10 ,0x02,0x54,0x44,0x43,0x74,0x51,0x70,0x81,0x33,0x26,
  10 ,0x01,0x27,0x22,0x21,0x87,0x25,0x85,0x40,0x66,0x63,
  9 ,0x63,0x61,0x10,0x93,0x62,0x92,0x70,0x33,0x31,
  9 ,0x31,0x80,0x55,0x46,0x81,0x46,0x35,0x16,0x66,
  9 ,0x15,0x90,0x27,0x73,0x40,0x73,0x17,0x58,0x33,
  9 ,0x07,0x95,0x13,0x86,0x70,0x36,0x58,0x79,0x16,
  9 ,0x03,0x97,0x56,0x93,0x35,0x18,0x29,0x39,0x58,
  9 ,0x01,0x98,0x78,0x46,0x67,0x59,0x14,0x69,0x79,
  8 ,0x99,0x39,0x23,0x33,0x79,0x57,0x34,0x90,
  8 ,0x49,0x69,0x61,0x66,0x89,0x78,0x67,0x45,
  8 ,0x24,0x84,0x80,0x83,0x44,0x89,0x33,0x72,
  8 ,0x12,0x42,0x40,0x41,0x72,0x44,0x66,0x86,
  8 ,0x06,0x21,0x20,0x20,0x86,0x22,0x33,0x43,
  8 ,0x03,0x10,0x60,0x10,0x43,0x11,0x16,0x72,
  8 ,0x01,0x55,0x30,0x05,0x21,0x55,0x58,0x36,
  7 ,0x77,0x65,0x02,0x60,0x77,0x79,0x18,
  7 ,0x38,0x82,0x51,0x30,0x38,0x89,0x59,
  7 ,0x19,0x41,0x25,0x65,0x19,0x44,0x79,
  7 ,0x09,0x70,0x62,0x82,0x59,0x72,0x40,
  7 ,0x04,0x85,0x31,0x41,0x29,0x86,0x20,
  7 ,0x02,0x42,0x65,0x70,0x64,0x93,0x10,
  7 ,0x01,0x21,0x32,0x85,0x32,0x46,0x55,
  6 ,0x60,0x66,0x42,0x66,0x23,0x27,
  6 ,0x30,0x33,0x21,0x33,0x11,0x64,
  6 ,0x15,0x16,0x60,0x66,0x55,0x82,
  6 ,0x07,0x58,0x30,0x33,0x27,0x91,
  6 ,0x03,0x79,0x15,0x16,0x63,0x95,
  6 ,0x01,0x89,0x57,0x58,0x31,0x98,
  5 ,0x94,0x78,0x79,0x15,0x99,
  5 ,0x47,0x39,0x39,0x57,0x99,
  5 ,0x23,0x69,0x69,0x79,0x00,
  5 ,0x11,0x84,0x84,0x89,0x50,
  5 ,0x05,0x92,0x42,0x44,0x75,
  5 ,0x02,0x96,0x21,0x22,0x37,
  5 ,0x01,0x48,0x10,0x61,0x19,
  4 ,0x74,0x05,0x30,0x59,
  4 ,0x37,0x02,0x65,0x30,
  4 ,0x18,0x51,0x32,0x65,
  4 ,0x09,0x25,0x66,0x32,
  4 ,0x04,0x62,0x83,0x16,
  4 ,0x02,0x31,0x41,0x58,
  4 ,0x01,0x15,0x70,0x79,
  3 ,0x57,0x85,0x40,
  3 ,0x28,0x92,0x70,
  3 ,0x14,0x46,0x35,
  3 ,0x07,0x23,0x17,
  3 ,0x03,0x61,0x59,
  3 ,0x01,0x80,0x79,
  2 ,0x90,0x40,
  2 ,0x45,0x20,
  2 ,0x22,0x60,
  2 ,0x11,0x30,
  2 ,0x05,0x65,
  2 ,0x02,0x82,
  2 ,0x01,0x41,
  1 ,0x71,
  1 ,0x35,
  1 ,0x18,
  1 ,0x09,
  1 ,0x04,
  1 ,0x02,
  1 ,0x01,
  0};

  int i,i_end;
  int table_ptr=0,log_ptr=0;
  do
  {
    logs[log_ptr+BCD_SIGN]=0;
    logs[log_ptr+BCD_DEC]=2;
    logs[log_ptr+BCD_LEN]=34;
    log_ptr+=3;
    i_end=table[table_ptr];
    table_ptr++;
    for (i=0;i<(17-i_end);i++)
    {
      logs[log_ptr++]=0;
      logs[log_ptr++]=0;
    }
    for (i=0;i<i_end;i++)
    {
      logs[log_ptr]=table[table_ptr]>>4;
      logs[log_ptr+1]=table[table_ptr]&0xF;
      table_ptr++;
      log_ptr+=2;
    }
  } while(table[table_ptr]);
}

static void SetDecPlaces()
{
  int i,j=2,x;
  //unsigned char which_backup=which_stack;

  //for (x=0;x<2;x++)
  //{
    //which_stack=x;

    j=2;
    for (i=0;i<MATH_TRIG_TABLE;i++)
    {
      trig[i*MATH_ENTRY_SIZE+BCD_LEN]=j+Settings.DecPlaces;
      if (IsZero(trig+i*MATH_ENTRY_SIZE)) break;
    }
    Settings.TrigTableSize=i+1;
    for (i=0;i<MATH_LOG_TABLE;i++)
    {
      logs[i*MATH_ENTRY_SIZE+BCD_LEN]=j+Settings.DecPlaces;
      if (IsZero(logs+i*MATH_ENTRY_SIZE)) break;
    }
    Settings.LogTableSize=i+1;//this was +0 on slave

    perm_K[BCD_LEN]=1+Settings.DecPlaces;
    perm_log10[BCD_LEN]=1+Settings.DecPlaces;
  //}
  //which_stack=which_backup;
}

static void BufferBCD_EtI(const unsigned char *text, unsigned char *BCD)
{
  #pragma MM_VAR text
  //#pragma MM_VAR BCD

  unsigned char *RAM_ptr;
  int BCD_ptr=3,text_ptr=0;
  char found=0;

  if (text[0]=='-')
  {
    text++;
    BCD[BCD_SIGN]=1;
  }
  else BCD[BCD_SIGN]=0;

  BCD[BCD_LEN]=0;
  BCD[BCD_DEC]=0;

  while (text[text_ptr])
  {
    if (text[text_ptr]=='.')
    {
      BCD[BCD_DEC]=text_ptr;
      found=1;
    }
    else
    {
      BCD[BCD_ptr]=text[text_ptr]-'0';
      BCD_ptr++;
    }
    text_ptr++;
  }
  BCD[BCD_LEN]=text_ptr-found;
  if (found==0) BCD[BCD_DEC]=BCD[BCD_LEN];
}

static void ImmedBCD(const char *text, unsigned char *BCD)
{
  int text_ptr=0;
  do
  {
    perm_buff2[text_ptr]=(unsigned char)text[text_ptr];
  } while(text[text_ptr++]);
  BufferBCD_ItE(perm_buff2,BCD);
}

static void BufferBCD_ItE(const unsigned char *text, unsigned char *BCD)
{
  //#pragma MM_VAR text
  #pragma MM_VAR BCD

  unsigned char *RAM_ptr;
  int BCD_ptr=3,text_ptr=0;
  char found=0;

  if (text[0]=='-')
  {
    text++;
    BCD[BCD_SIGN]=1;
  }
  else BCD[BCD_SIGN]=0;

  BCD[BCD_LEN]=0;
  BCD[BCD_DEC]=0;

  while (text[text_ptr])
  {
    if (text[text_ptr]=='.')
    {
      BCD[BCD_DEC]=text_ptr;
      found=1;
    }
    else
    {
      BCD[BCD_ptr]=text[text_ptr]-'0';
      BCD_ptr++;
    }
    text_ptr++;
  }
  BCD[BCD_LEN]=text_ptr-found;
  if (found==0) BCD[BCD_DEC]=BCD[BCD_LEN];
}

static void ImmedBCD_RAM(const char *text, unsigned char *BCD)
{
  int text_ptr=0;
  do
  {
    perm_buff2[text_ptr]=(unsigned char)text[text_ptr];
  } while(text[text_ptr++]);
  BufferBCD_ItI(perm_buff2,BCD);
}

static void BufferBCD_ItI(const unsigned char *text, unsigned char *BCD)
{
  //#pragma MM_VAR text
  //#pragma MM_VAR BCD

  unsigned char *RAM_ptr;
  int BCD_ptr=3,text_ptr=0;
  char found=0;

  if (text[0]=='-')
  {
    text++;
    BCD[BCD_SIGN]=1;
  }
  else BCD[BCD_SIGN]=0;

  BCD[BCD_LEN]=0;
  BCD[BCD_DEC]=0;

  while (text[text_ptr])
  {
    if (text[text_ptr]=='.')
    {
      BCD[BCD_DEC]=text_ptr;
      found=1;
    }
    else
    {
      BCD[BCD_ptr]=text[text_ptr]-'0';
      BCD_ptr++;
    }
    text_ptr++;
  }
  BCD[BCD_LEN]=text_ptr-found;
  if (found==0) BCD[BCD_DEC]=BCD[BCD_LEN];
}

static bool IsZero(unsigned char *n1)
{
  #pragma MM_VAR n1
  int i,i_end;
  i_end=n1[BCD_LEN]+3;
  for (i=3;i<i_end;i++) if (n1[i]!=0) return false;
  return true;
}

static bool IsZero_RAM(unsigned char *n1)
{
  //#pragma MM_VAR n1
  int i,i_end;
  i_end=n1[BCD_LEN]+3;
  for (i=3;i<i_end;i++) if (n1[i]!=0) return false;
  return true;
}

static void AddBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR n1
  //#pragma MM_VAR n2

  unsigned char carry;
  const unsigned char *temp;
  unsigned char sign;
  //pointer to loop through number, length of BCD so don't have to keep checking
  int BCD_ptr, BCD_end;
  //where to stop counting if whole parts are different
  int n1_whole=0, n2_whole=0;
  //where to start counting if decimal lengths are different
  int n1_dec=0, n2_dec=0;
  //number to add when carrying. set to 9 for subtraction.
  unsigned char carry_number=0;
  bool subtracting=false;
  int t1,t2,d1,d2;

  t1=n1[BCD_SIGN];
  t2=n2[BCD_SIGN];

  if ((t1==0)&&(t2==0)) sign=0;
  else if ((t1==1)&&(t2==1)) sign=1;
  else sign=2;

  if ((t1==1)&&(t2==0))
  {
    temp=n1;
    n1=n2;
    n2=temp;
  }

  if ((n1[BCD_SIGN]==0)&&(n2[BCD_SIGN]==1))
  {
    buffer[BCD_DEC]=n2[BCD_DEC];
    buffer[BCD_LEN]=n2[BCD_LEN];
    carry=1;
    for (BCD_ptr=n2[BCD_LEN]+2;BCD_ptr>=3;BCD_ptr--)
    {
      t1=9-n2[BCD_ptr]+carry;
      if (t1==10) buffer[BCD_ptr]=0;
      else
      {
        carry=0;
        buffer[BCD_ptr]=t1;
      }
    }
    carry_number=9;
    if (carry==1)
    {
      //buffer[3]=0;
      carry_number=0;
    }
    n2=buffer;
    subtracting=true;
  }

  //make result whole part equal to the greater of operand whole parts + 1
  t1=n1[BCD_DEC];
  t2=n2[BCD_DEC];
  if (t1>t2)
  {
    n2_whole=t1-t2;
    result[BCD_DEC]=t1;
  }
  else
  {
    n1_whole=t2-t1;
    result[BCD_DEC]=t2;
  }

  //make decimal length equal to the greater of operand decimal lengths
  d1=n1[BCD_LEN];
  d2=n2[BCD_LEN];

  if ((d1-t1)>(d2-t2))
  {
    n2_dec=(d1-t1)-(d2-t2);
    result[BCD_LEN]=result[BCD_DEC]+d1-t1;
  }
  else
  {
    n1_dec=(d2-t2)-(d1-t1);
    result[BCD_LEN]=result[BCD_DEC]+d2-t2;
  }

  carry=0;
  BCD_end=result[BCD_LEN]+2;
  for (BCD_ptr=BCD_end;BCD_ptr>=3;BCD_ptr--)
  {
    t1=carry;
    if ((BCD_ptr<=BCD_end-n1_dec)&&(BCD_ptr>n1_whole+2)) t1+=n1[BCD_ptr-n1_whole];
    if ((BCD_ptr<=BCD_end-n2_dec)&&(BCD_ptr>n2_whole+2)) t1+=n2[BCD_ptr-n2_whole];
    if (BCD_ptr<=n2_whole+2) t1+=carry_number;

    if (t1>9)
    {
      t1-=10;
      carry=1;
    }
    else carry=0;
    result[BCD_ptr]=t1;
  }

  if ((carry==1)&&(subtracting==false))
  {
    PadBCD_RAM(result,1);
    result[3]=1;
  }

  if ((carry==0)&&(carry_number==9)&&(sign==2))
  {
    carry=1;
    for (BCD_ptr=result[BCD_LEN]+2;BCD_ptr>=3;BCD_ptr--)
    {
      t1=9-result[BCD_ptr]+carry;
      if (t1==10) t1=0;
      else carry=0;
      result[BCD_ptr]=t1;
    }
    sign=1;
  }
  else if (sign==2) sign=0;
  result[BCD_SIGN]=sign;
}

static void SubBCD(unsigned char *result, const unsigned char *n1, unsigned char *n2)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR n1
  //#pragma MM_VAR n2
  n2[BCD_SIGN]=!n2[BCD_SIGN];
  AddBCD(result,n1,n2);
  n2[BCD_SIGN]=!n2[BCD_SIGN];
}

static void PrintBCD(const unsigned char *BCD, int dec_point)
{
  #pragma MM_VAR BCD
  int BCD_ptr,BCD_end;
  bool zero=true;

  BCD_end=BCD[BCD_LEN]+3;
  if (dec_point>=0)
  {
    if ((BCD[BCD_LEN]-BCD[BCD_DEC])>dec_point)
    {
      BCD_end=BCD[BCD_DEC]+dec_point+3;
    }
  }

  if (BCD[BCD_SIGN]==1) putchar('-');
  for (BCD_ptr=3;BCD_ptr<BCD_end;BCD_ptr++)
  {
    if (BCD_ptr==BCD[BCD_DEC]+3) putchar('.');
    if (BCD[BCD_ptr]>9) putchar('x');
    else putchar('0'+BCD[BCD_ptr]);
  }
}

static void PrintBCD_RAM(const unsigned char *BCD, int dec_point)
{
  //#pragma MM_VAR BCD
  int BCD_ptr,BCD_end;
  bool zero=true;

  BCD_end=BCD[BCD_LEN]+3;
  if (dec_point>=0)
  {
    if ((BCD[BCD_LEN]-BCD[BCD_DEC])>dec_point)
    {
      BCD_end=BCD[BCD_DEC]+dec_point+3;
    }
  }

  if (BCD[BCD_SIGN]==1) putchar('-');
  for (BCD_ptr=3;BCD_ptr<BCD_end;BCD_ptr++)
  {
    if (BCD_ptr==BCD[BCD_DEC]+3) putchar('.');
    if (BCD[BCD_ptr]>9) putchar('x');
    else putchar('0'+BCD[BCD_ptr]);
  }
}

static void MultBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR n1
  //#pragma MM_VAR n2
  //#pragma MM_VAR temp

  //#pragma MM_DECLARE
    unsigned char temp[5];
  //#pragma MM_END

  unsigned char i,j,k,flip=0;
  unsigned char i_end, j_end, k_end;
  //maybe b1,b2 is faster
  unsigned char b0,b1;
  CopyBCD_ItI(result,perm_zero);
  CopyBCD_ItI(perm_buff1,perm_zero);

  temp[BCD_SIGN]=0;
  temp[BCD_LEN]=2;

  i_end=n1[BCD_LEN];
  j_end=n2[BCD_LEN];
  for (i=0;i<i_end;i++)
  {
    for (j=0;j<j_end;j++)
    {
      b0=0;
      b1=0;
      k_end=n1[i+3];
      for (k=0;k<k_end;k++) b0+=n2[j+3];
      while (b0>9)
      {
        b1+=1;
        b0-=10;
      }
      temp[3]=b1;
      temp[4]=b0;

      temp[BCD_DEC]=2+i_end-i+j_end-j-2;
      if (flip==0) AddBCD(result,temp,perm_buff1);
      else AddBCD(perm_buff1,temp,result);
      flip=!flip;
    }
  }

  if (flip==0)
  {
    //Just CopyBCD?
    i=perm_buff1[BCD_LEN];
    for (j=0;j<i+3;j++) result[j]=perm_buff1[j];
  }
  i=(i_end-n1[BCD_DEC])+(j_end-n2[BCD_DEC]);

  if (i>Settings.DecPlaces)
  {
    result[BCD_LEN]-=(i-Settings.DecPlaces-1);
    result[BCD_DEC]=result[BCD_LEN];

    if (result[result[BCD_LEN]+2]>4)
    {
      ImmedBCD_RAM("10",temp);
      AddBCD(perm_buff1,result,temp);
      CopyBCD_ItI(result,perm_buff1);
    }
    result[BCD_LEN]-=1;
    i=Settings.DecPlaces+1;
  }
  result[BCD_DEC]-=i;
  result[BCD_SIGN]=n1[BCD_SIGN]^n2[BCD_SIGN];

  FullShrinkBCD_RAM(result);
}

static void DivBCD(unsigned char *result, const unsigned char *n1, const unsigned char *n2)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR n1
  //#pragma MM_VAR n2

  int i,j;
  int i_end, j_end;
  int result_ptr,n1_ptr;
  int max_offset,dec_offset=0;
  int post_offset=0, pre_offset=0;
  unsigned char remainder=0, res_ptr_off=0;
  bool logic;

  max_offset=n1[BCD_LEN]-n1[BCD_DEC];
  if ((n2[BCD_LEN]-n2[BCD_DEC])>max_offset) max_offset=n2[BCD_LEN]-n2[BCD_DEC];
  if (Settings.DecPlaces>max_offset) max_offset=Settings.DecPlaces;

  //result[BCD_SIGN]=0;
  result[BCD_LEN]=0;

  result_ptr=3;

  post_offset=n1[BCD_LEN]-n2[BCD_LEN];

  if ((n1[BCD_DEC]-n2[BCD_DEC]+1)<post_offset)
  {
    pre_offset=-1*(n1[BCD_DEC]-n2[BCD_DEC]+1);
    post_offset=0;
    if (pre_offset<0)
    {
      result[BCD_DEC]=-pre_offset;
    }
    else
    {
      result[BCD_DEC]=0;
      result_ptr+=pre_offset;
      res_ptr_off+=pre_offset;
      for (i=3;i<(pre_offset+3);i++) result[i]=0;
    }
  }
  else if (post_offset>0)
  {
    post_offset=0;
    result[BCD_DEC]=n1[BCD_DEC]-n2[BCD_DEC]+1;
  }
  else
  {
    post_offset*=-1;
    result[BCD_DEC]=post_offset+n1[BCD_DEC]-n2[BCD_DEC]+1;
  }

  perm_buff2[BCD_SIGN]=1;
  perm_buff2[BCD_LEN]=n2[BCD_LEN];
  perm_buff2[BCD_DEC]=n2[BCD_LEN];

  perm_buff1[BCD_SIGN]=0;
  perm_buff1[BCD_LEN]=n2[BCD_LEN]+1;
  perm_buff1[BCD_DEC]=perm_buff1[BCD_LEN];
  perm_buff1[3]=0;

  i_end=n2[BCD_LEN]+3;
  for (i=3;i<i_end;i++)
  {
    //<=? was <
    if ((i-3)<post_offset) perm_buff1[i+1]=0;
    else if ((i-post_offset)>(n1[BCD_LEN]+2)) perm_buff1[i+1]=0;
    else perm_buff1[i+1]=n1[i-post_offset];
    perm_buff2[i]=n2[i];
  }

  n1_ptr=n2[BCD_LEN]+3+post_offset;
  do
  {
    result[result_ptr]=0;
    result[BCD_LEN]+=1;

    do
    {
      AddBCD(perm_buff3,perm_buff1,perm_buff2);

      if ((perm_buff3[BCD_SIGN]==0)||(IsZero_RAM(perm_buff3)))
      {
        result[result_ptr]+=1;
        if (result[result_ptr]==10)
        {
          result[result_ptr]=0;
          for (i=result_ptr-1;i>=3;i--)
          {
            result[i]+=1;
            if (result[i]<10) break;
            else result[i]=0;
          }
          if (i==2)
          {
            result[3]=1;
            for (i=4;i<result_ptr;i++) result[i]=0;
            result_ptr++;
            result[BCD_LEN]+=1;
            result[BCD_DEC]+=1;
            result[result_ptr]=0;
          }
        }
        i_end=perm_buff1[BCD_LEN]+3;
        for (i=3;i<i_end;i++) perm_buff1[i]=perm_buff3[i];
      }
    } while ((perm_buff3[BCD_SIGN]==0)&&(!IsZero_RAM(perm_buff3)));

    i_end=n2[BCD_LEN]+3;
    for (i=3;i<i_end;i++) perm_buff1[i]=perm_buff1[i+1];

    if ((n1_ptr-3)>=n1[BCD_LEN])
    {
      perm_buff1[i]=0;
    }
    else
    {
      perm_buff1[i]=n1[n1_ptr];
      n1_ptr++;
    }
    result_ptr++;

    //< or <=?
    if ((result_ptr-3)<(result[BCD_DEC]))
    {
      logic=true;
    }
    else if (((result[BCD_LEN]-result[BCD_DEC])<(max_offset+1)))//&&(!IsZero(div_buff1)))
    {
      logic=true;
    }
    else logic=false;
  } while (logic);

  if ((result[BCD_LEN]-result[BCD_DEC])>max_offset)
  {
    if (result[result[BCD_LEN]+2]>4)
    {
      i_end=result[BCD_LEN]+2;
      for (i=3;i<i_end;i++) perm_buff3[i]=result[i];
      i=result[BCD_LEN];
      j=result[BCD_DEC];
      perm_buff3[BCD_SIGN]=0;
      perm_buff3[BCD_LEN]=result[BCD_LEN]-1;
      perm_buff3[BCD_DEC]=perm_buff3[BCD_LEN];
      perm_buff1[BCD_SIGN]=0;
      perm_buff1[BCD_LEN]=1;
      perm_buff1[BCD_DEC]=1;
      perm_buff1[3]=1;
      AddBCD(result,perm_buff3,perm_buff1);
      result[BCD_DEC]=j;
      if (result[BCD_LEN]==i) result[BCD_DEC]+=1;
    }
    else
    {
      result[BCD_LEN]-=1;
    }
  }
  result[BCD_SIGN]=n1[BCD_SIGN]^n2[BCD_SIGN];
  FullShrinkBCD_RAM(result);
}

static void ShrinkBCD(unsigned char *dest,unsigned char *src)
{
  #pragma MM_VAR dest
  #pragma MM_VAR src

  int BCD_ptr,off_ptr=0;
  int ptr_end;
  if ((src[3]==0)&&(src[BCD_DEC]!=0)&&(src[BCD_LEN]>1)) off_ptr=1;
  ptr_end=src[BCD_LEN]+3-off_ptr;
  for (BCD_ptr=3;BCD_ptr<ptr_end;BCD_ptr++) dest[BCD_ptr]=src[BCD_ptr+off_ptr];
  dest[BCD_SIGN]=src[BCD_SIGN];
  dest[BCD_LEN]=src[BCD_LEN];
  dest[BCD_DEC]=src[BCD_DEC];
  if (off_ptr==1)
  {
    dest[BCD_LEN]-=1;
    dest[BCD_DEC]-=1;
  }
}

static void ShrinkBCD_RAM(unsigned char *dest,unsigned char *src)
{
  //#pragma MM_VAR dest
  //#pragma MM_VAR src

  int BCD_ptr,off_ptr=0;
  int ptr_end;
  if ((src[3]==0)&&(src[BCD_DEC]!=0)&&(src[BCD_LEN]>1)) off_ptr=1;
  ptr_end=src[BCD_LEN]+3-off_ptr;
  for (BCD_ptr=3;BCD_ptr<ptr_end;BCD_ptr++) dest[BCD_ptr]=src[BCD_ptr+off_ptr];
  dest[BCD_SIGN]=src[BCD_SIGN];
  dest[BCD_LEN]=src[BCD_LEN];
  dest[BCD_DEC]=src[BCD_DEC];
  if (off_ptr==1)
  {
    dest[BCD_LEN]-=1;
    dest[BCD_DEC]-=1;
  }
}

static void FullShrinkBCD(unsigned char *n1)
{
  #pragma MM_VAR n1
  while ((n1[3]==0)&&(n1[BCD_DEC]>1)) ShrinkBCD(n1,n1);
}

static void FullShrinkBCD_RAM(unsigned char *n1)
{
  //#pragma MM_VAR n1
  while ((n1[3]==0)&&(n1[BCD_DEC]>1))
  {
    ShrinkBCD_RAM(n1,n1);
  }
}

static void PadBCD(unsigned char *n1, int amount)
{
  #pragma MM_VAR n1
  int i;
  for (i=n1[BCD_LEN]+3;i>=3;i--) n1[i+amount]=n1[i];
  for (i=3;i<(amount+3);i++) n1[i]=0;
  n1[BCD_LEN]+=amount;
  n1[BCD_DEC]+=amount;
}

static void PadBCD_RAM(unsigned char *n1, int amount)
{
  //#pragma MM_VAR n1
  int i;
  for (i=n1[BCD_LEN]+3;i>=3;i--) n1[i+amount]=n1[i];
  for (i=3;i<(amount+3);i++) n1[i]=0;
  n1[BCD_LEN]+=amount;
  n1[BCD_DEC]+=amount;
}

//see if using this in other places makes things smaller
static void CopyBCD(unsigned char *dest, unsigned char *src)
{
  #pragma MM_VAR dest
  #pragma MM_VAR src
  int i,i_end;
  i_end=src[BCD_LEN]+3;
  for (i=0;i<i_end;i++) dest[i]=src[i];
}

static void CopyBCD_EtI(unsigned char *dest, unsigned char *src)
{
  //#pragma MM_VAR dest
  #pragma MM_VAR src
  int i,i_end;
  i_end=src[BCD_LEN]+3;
  for (i=0;i<i_end;i++) dest[i]=src[i];
}

static void CopyBCD_ItE(unsigned char *dest, unsigned char *src)
{
  #pragma MM_VAR dest
  //#pragma MM_VAR src
  int i,i_end;
  i_end=src[BCD_LEN]+3;
  for (i=0;i<i_end;i++) dest[i]=src[i];
}

static void CopyBCD_ItI(unsigned char *dest, unsigned char *src)
{
  //#pragma MM_VAR dest
  //#pragma MM_VAR src
  int i,i_end;
  i_end=src[BCD_LEN]+3;
  for (i=0;i<i_end;i++) dest[i]=src[i];
}

static bool LnBCD(unsigned char *result, unsigned char *arg)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR arg
  //#pragma MM_VAR temp

  //#pragma MM_DECLARE
    unsigned char temp[4];
  //#pragma MM_END

  bool flip_sign=false;
  unsigned int i,j=1,k=0;

  ImmedBCD_RAM("1",temp);

  SubBCD(p1,arg,temp);
  //CopyBCD_EtI(p0,arg);
  //SubBCD(p1,p0,temp);
  if (IsZero_RAM(p1))
  {
    CopyBCD_ItI(result,perm_zero);
    return true;
  }
  else if (p1[BCD_SIGN]==1)
  {
    DivBCD(p1,temp,arg);
    //CopyBCD_EtI(p0,arg);
    //DivBCD(p1,temp,p0);
    flip_sign=true;
  }
  else CopyBCD_ItI(p1,arg);

  for (i=0;i<8;i++)
  {
    RorBCD(p0,p1,j);
    CopyBCD_ItI(p1,p0);
    SubBCD(p0,p1,temp);
    if (p0[BCD_SIGN]==1) break;
    j=1<<(k++);
  }

  if (i==8) return false;
  if (IsZero_RAM(p1)) return false;

  j=1<<i;
  k=7-k;
  CopyBCD_EtI(result,logs+k*MATH_ENTRY_SIZE);

  for (i=k;i<Settings.LogTableSize;i++)
  {
    if (j!=0)
    {
      RolBCD(p0,p1,j);
      SubBCD(p2,p0,temp);
      j>>=1;
    }
    else
    {
      RorBCD(p2,p1,i-7);
      AddBCD(p0,p1,p2);
      SubBCD(p2,p0,temp);
    }
    if (p2[BCD_SIGN]==1)
    {
      CopyBCD_ItI(p1,p0);
      //temp for SubBCD
      CopyBCD_EtI(p0,logs+i*MATH_ENTRY_SIZE);
      //SubBCD(p2,result,logs+i*MATH_ENTRY_SIZE);
      SubBCD(p2,result,p0);
      CopyBCD_ItI(result,p2);
    }
  }
  SubBCD(p2,temp,p1);
  SubBCD(p0,result,p2);
  CopyBCD_ItI(result,p0);
  if (flip_sign) result[BCD_SIGN]=1;
  return true;
}

//ExpBCD(stack_buffer,BCD_stack+(stack_ptr[which_stack]-1)*MATH_CELL_SIZE);
static void ExpBCD(unsigned char *result, unsigned char *arg)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR arg
  //#pragma MM_VAR temp

  //#pragma MM_DECLARE
    unsigned char temp[4];
  //#pragma MM_END

  unsigned int i,j=128;
  unsigned int log_ptr=0;
  bool invert=false;
  if (arg[BCD_SIGN]==1)
  {
    invert=true;
    arg[BCD_SIGN]=0;
  }

  /*if (CompVarBCD(perm_zero,arg)==COMP_EQ)
  {
    ImmedBCD_RAM("1",result);
    return;
  }*/

  if (IsZero_RAM(arg))
  {
    ImmedBCD_RAM("1",result);
    return;
  }

  ImmedBCD_RAM("1",temp);
  CopyBCD_ItI(p0,arg);
  CopyBCD_ItI(result,temp);//stack_buffer
  for (i=0;i<Settings.LogTableSize;i++)
  {
    CopyBCD_EtI(perm_buff1,logs+log_ptr);
    //SubBCD(p1,p0,logs+log_ptr);
    SubBCD(p1,p0,perm_buff1);
    if (p1[BCD_SIGN]==0)
    {
      CopyBCD_ItI(p0,p1);
      if (i<8)
      {
        RolBCD(p1,result,j);
      }
      else
      {
        RorBCD(p2,result,i-7);
        AddBCD(p1,result,p2);
      }
      CopyBCD_ItI(result,p1);
    }
    j>>=1;
    log_ptr+=MATH_ENTRY_SIZE;
  }
  AddBCD(p1,p0,temp);
  MultBCD(p2,p1,result);
  CopyBCD_ItI(p2,result);

  if (invert) DivBCD(result,temp,p2);
  else CopyBCD_ItI(result,p2);
}

static void RolBCD(unsigned char *result, unsigned char *arg, unsigned char amount)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR arg

  int i,i_end,j;
  unsigned char b0,b1;

  CopyBCD_ItI(result,arg);
  for (j=0;j<amount;j++)
  {
    b1=0;
    i_end=result[BCD_LEN]+2;
    for (i=i_end;i>=3;i--)
    {
      b0=(result[i]<<1)+b1;
      if (b0>9) b0+=6;
      b1=b0>>4;
      result[i]=(b0&0xF);
    }
    if (b1)
    {
      PadBCD_RAM(result,1);
      result[3]=1;
    }
  }
}

//Does shifting one bit add an extra 0?
static void RorBCD(unsigned char *result, unsigned char *arg, unsigned char amount)
{
  //#pragma MM_VAR result
  //#pragma MM_VAR arg

  int i,i_end,j;
  unsigned char b0,b1,b2;
  unsigned char t1;

  static const unsigned char table[]={
  0,0,0,0, // 0/16
  0,6,2,5, // 1/16
  1,2,5,0, // 2/16
  1,8,7,5, // 3/16
  2,5,0,0, // 4/16
  3,1,2,5, // 5/16
  3,7,5,0, // 6/16
  4,3,7,5, // 7/16
  5,0,0,0, // 8/16
  5,6,2,5};// 9/16

  unsigned char accum[6];

  CopyBCD_ItI(result,arg);

  while (amount)
  {
    if (amount>3)
    {
      for (i=0;i<6;i++) accum[i]=0;
      amount-=4;

      t1=result[BCD_DEC];
      b0=0;
      i_end=result[BCD_LEN]+3;
      for (i=3;i<i_end;i++)
      {
        b1=0;
        b2=result[i]*4;
        for (j=3;j>=-1;j--)
        {
          if (j>=0) accum[b0+j]+=table[b2+j]+b1;
          else accum[b0+j]+=b1;

          if (accum[b0+j]>9)
          {
            accum[b0+j]-=10;
            b1=1;
          }
          else b1=0;
        }

        if (b0!=2) b0++;
        else
        {
          if (t1) result[i-2]=accum[0];
          else result[i-1]=accum[0];

          for (j=0;j<5;j++) accum[j]=accum[j+1];
          accum[5]=0;
        }
      }

      b1=result[BCD_LEN]+3;
      if (t1)
      {
        t1--;
        result[BCD_LEN]=b1;
        result[BCD_DEC]=t1;
        b2=i-b0;
      }
      else
      {
        b1++;
        result[3]=0;
        result[BCD_LEN]=b1;
        b2=i-b0+1;
      }
      for (j=0;j<5;j++) result[j+b2]=accum[j];

      if ((b1-t1)>Settings.DecPlaces) result[BCD_LEN]=t1+Settings.DecPlaces;
    }
    else
    {
      amount--;
      i_end=result[BCD_LEN]+3;
      b1=0;
      for (i=3;i<i_end;i++)
      {
        b0=result[i];
        b2=b0;
        b0=(b0>>1)+b1;
        b1=0;
        if (b2&1) b1=8;
        if (b0>7) b0-=3;
        result[i]=b0;
      }
      if (b1)
      {
        if((result[BCD_LEN]-result[BCD_DEC])<Settings.DecPlaces)
        {
          b1=result[BCD_LEN]+1;
          result[BCD_LEN]=b1;
          result[b1+2]=5;
        }
      }
    }
  }
}

//result-internal, base-external, exp-internal
static void PowBCD(unsigned char *result, unsigned char *base, unsigned char *exp)
{
  LnBCD(p3,base);
  MultBCD(p4,p3,exp);
  ExpBCD(result,p4);
}

static void TanBCD(unsigned char *sine_result,unsigned char *cos_result,unsigned char *arg)
{
  //#pragma MM_VAR sine_result
  //#pragma MM_VAR cos_result

  CopyBCD_ItI(p2,perm_zero);
  CopyBCD_ItI(sine_result,perm_zero);
  CopyBCD_ItI(cos_result,perm_K);

  CalcTanBCD(sine_result,cos_result,p2,arg,0);
  sine_result[BCD_LEN]=Settings.DecPlaces;
  cos_result[BCD_LEN]=Settings.DecPlaces;
}

static void AcosBCD(unsigned char *result,unsigned char *arg)
{
  CopyBCD_ItI(p0,arg);
  MultBCD(p1,p0,arg);
  ImmedBCD_RAM("1",p0);
  SubBCD(p5,p0,p1);
  ImmedBCD_RAM("0.5",p6);
  PowBCD(p7,p5,p6);
  DivBCD(p6,p7,arg);
  AtanBCD(result,p6);
}

static void AsinBCD(unsigned char *result,unsigned char *arg)
{
  CopyBCD_ItI(p0,arg);
  MultBCD(p1,p0,arg);
  ImmedBCD_RAM("1",p0);
  SubBCD(p5,p0,p1);
  ImmedBCD_RAM("0.5",p6);
  PowBCD(p7,p5,p6);
  DivBCD(p6,arg,p7);
  AtanBCD(result,p6);
}

static void AtanBCD(unsigned char *result,unsigned char *arg)
{
  //#pragma MM_VAR result

  CopyBCD_ItI(result,perm_zero);
  ImmedBCD_RAM("1",p2);
  CopyBCD_ItI(p3,arg);
  CalcTanBCD(p2,p3,result,arg,1);

  if ((result[BCD_DEC]<=Settings.DecPlaces)&&(result[BCD_LEN]>Settings.DecPlaces)) result[BCD_LEN]=Settings.DecPlaces;
}

static void CalcTanBCD(unsigned char *result1,unsigned char *result2,unsigned char *result3,unsigned char *arg,unsigned char flag)
{
  //#pragma MM_VAR result2

  unsigned int i;
  unsigned int trig_ptr=0;

  //function pointers could reduce flash size
  for (i=0;i<Settings.TrigTableSize;i++)
  {
    #define DEBUGTIME 0
    if (flag==0) SubBCD(p1,arg,result3);

    if (((flag==0)&&(p1[BCD_SIGN]==0))||((flag==1)&&(result2[BCD_SIGN]==0)))
    {
      RorBCD(p0,result2,i);
      AddBCD(p1,result1,p0);
      RorBCD(p0,result1,i);
      CopyBCD_ItI(result1,p1);
      SubBCD(p1,result2,p0);
      CopyBCD_ItI(result2,p1);
      CopyBCD_EtI(perm_buff1,trig+trig_ptr);
      AddBCD(p0,result3,perm_buff1);
    }
    else
    {
      RorBCD(p0,result2,i);
      SubBCD(p1,result1,p0);
      RorBCD(p0,result1,i);
      CopyBCD_ItI(result1,p1);
      AddBCD(p1,result2,p0);
      CopyBCD_ItI(result2,p1);
      CopyBCD_EtI(perm_buff1,trig+trig_ptr);
      SubBCD(p0,result3,perm_buff1);
    }

    CopyBCD_ItI(result3,p0);
    trig_ptr+=MATH_ENTRY_SIZE;
  }
}

//var is external???
static unsigned char CompBCD(const char *num, unsigned char *var)
{
  ImmedBCD_RAM(num,p0);
  CopyBCD_EtI(local_buff1,var);
  return CompVarBCD(p0,local_buff1);
}

static unsigned char CompBCD_RAM(const char *num, unsigned char *var)
{
  ImmedBCD_RAM(num,p0);
  return CompVarBCD_ItI(p0,var);
}

//Should fail if ever called since cant SubBCD on externals
static unsigned char CompVarBCD(unsigned char *var1, unsigned char *var2)
{
  SubBCD(p1,var1,var2);
  if (IsZero_RAM(p1)) return COMP_EQ;
  else if (p1[BCD_SIGN]==0) return COMP_GT;
  else return COMP_LT;
}

static unsigned char CompVarBCD_ItE(unsigned char *var1, unsigned char *var2)
{
  /*SubBCD(p1,var1,var2);
  if (IsZero_RAM(p1)) return COMP_EQ;
  else if (p1[BCD_SIGN]==0) return COMP_GT;
  else return COMP_LT;*/
  return 0;
}

static unsigned char CompVarBCD_ItI(unsigned char *var1, unsigned char *var2)
{
  SubBCD(p1,var1,var2);
  if (IsZero_RAM(p1)) return COMP_EQ;
  else if (p1[BCD_SIGN]==0) return COMP_GT;
  else return COMP_LT;
}



//convert angle to 0-90 format and put in p3
static unsigned char TrigPrep(unsigned int stack_ptr_copy,int *cosine)
{
  int sine;

  if (Settings.DegRad) CopyBCD_EtI(p3,BCD_stack+(stack_ptr_copy-1)*MATH_CELL_SIZE);
  else
  {
    ImmedBCD_RAM(deg_factor,p0);
    //p1 should be free?
    CopyBCD_EtI(p1,BCD_stack+(stack_ptr_copy-1)*MATH_CELL_SIZE);
    MultBCD(p3,p1,p0);
  }

  ImmedBCD_RAM("360",p2);
  //Copy p1 since it has left over from CompVarBCD
  while(CompVarBCD_ItI(p3,p2)==COMP_GT) CopyBCD(p3,p1);

  if (CompBCD_RAM("180",p3)==COMP_LT)
  {
    SubBCD(stack_buffer,p2,p3);
    sine=1;
  }
  else
  {
    CopyBCD_ItI(stack_buffer,p3);
    sine=0;
  }
  if (CompBCD_RAM("90",stack_buffer)==COMP_LT)
  {
    ImmedBCD_RAM("180",p2);
    SubBCD(p3,p2,stack_buffer);
    *cosine=1;
  }
  else
  {
    CopyBCD_ItI(p3,stack_buffer);
    *cosine=0;
  }

  return sine;
}

static int ProgLine(int prog, int line, int start, bool fill_buff)
{
  int i;
  int line_counter=0, ptr;
  unsigned char retval;
  bool recording=false;
  bool stop_recording=false;
  //unsigned char record_buff;
  int record_ptr=0;
  bool finished=false;
  bool to_increase=false;
  int ret_ptr;

  //return 0;

  p0[0]=0;
  if (fill_buff) p1[0]=0;
    //ClrLCD();

  //clrscr();
  //gotoxy(-1,-1);

  //printf("Line: %d\n",line);


  //if first char is operator
  /*if (line==0)
  {
    //printf("Recording\n");
    recording=true;
    record_ptr=0;
    ret_ptr=ptr;
  }
  else*/
  ret_ptr=prog*PROG_TOTAL+PROG_HEADER;

  for (ptr=prog*PROG_TOTAL+PROG_HEADER;ptr<(prog+1)*PROG_TOTAL;ptr++)
  {
    which_stack=1;
    retval=RAM_Read((unsigned char *)ptr);
    which_stack=0;

    //printf("%d: %d",ptr-(prog*PROG_TOTAL+PROG_HEADER),retval);

    switch (retval)
    {
      case 0:
        finished=true;
        stop_recording=true;
        //added after restructure
        if (fill_buff) ProgLineFill(p1);
        if (recording) return ret_ptr;
        else return ptr;
        break;
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
      case '.':
        //record_buff=retval;
        break;
      case '+':
      case '-':
      case '*':
      case '/':
      case KEY_DUPE:
      case KEY_SWAP:
      case KEY_SQRT:
      case KEY_XRTY:
      case KEY_MOD:
      case KEY_COS:
      case KEY_ACOS:
      case KEY_EX:
      case KEY_10X:
      case KEY_LN:
      case KEY_LOG:
      case KEY_1X:
      case KEY_ROUND:
      case KEY_POW:
      case KEY_SIN:
      case KEY_ASIN:
      case KEY_TAN:
      case KEY_ATAN:
      case KEY_X2:
      case KEY_ENTER:
      case KEY_SIGN:
      case KEY_INS:
        //record_buff=retval;
        stop_recording=true;
        break;
      default:

        break;
    }

    if (stop_recording)
    {
      if (recording)
      {
        if (record_ptr!=0)
        {
          if (fill_buff) ProgLineFill(p1);
          return ret_ptr;
        }
        else
        {
          //printf("Done. WHAT HAPPENED?");
          //GetKey();
        }
        //return ret_ptr;
      }
      //what to do if very first char is operator and requested line 0?
      //if (!((line_counter==0)&&((line==0))))
      //{
        //seems to work if returning first char
        //also works fine if number is first
      if (ptr!=prog*PROG_TOTAL+PROG_HEADER)
      {
        line_counter++;
      }
      to_increase=true;
    }
    else if (to_increase)
    {
      //printf(" INCREASE ");
      line_counter++;
      to_increase=false;
    }

    //printf(" Line: %d\n",line_counter);

    if ((line_counter>=line)&&(recording==false))
    {
      //printf("Recording\n");
      recording=true;
      record_ptr=0;
      ret_ptr=ptr;
    }

    if (recording)
    {
      p0[record_ptr]=retval;
      record_ptr++;
      p0[record_ptr]=0;
      //means it was a stop char but buff was empty
      if (stop_recording)
      {
        //printf("Done. Empty: ");
        //for (i=0;i<10;i++) printf("%d ",p0[i]);
        //GetKey();
        //printf("*-*");
        if (fill_buff) ProgLineFill(p1);
        return ptr;
      }
    }
    stop_recording=false;
    //printf(".");
    //actually should never reach this!
    if (finished) return ret_ptr;
  }
  //printf("RAN OUT OF SPACE?");
  //GetKey();
  return 0;
}

static void ProgLineFill(unsigned char *buffer)
{
  int i;
  if (p0[0]==KEY_ENTER) ProgLineFillCopy(buffer,"ENTER");
  else if (p0[0]==KEY_DUPE) ProgLineFillCopy(buffer,"dupe");
  else if (p0[0]==KEY_SWAP) ProgLineFillCopy(buffer,"swap");
  else if (p0[0]==KEY_SQRT) ProgLineFillCopy(buffer,"sq.root");
  else if (p0[0]==KEY_XRTY) ProgLineFillCopy(buffer,"x root y");
  else if (p0[0]==KEY_MOD) ProgLineFillCopy(buffer,"\\");
  else if (p0[0]==KEY_COS) ProgLineFillCopy(buffer,"cos");
  else if (p0[0]==KEY_ACOS) ProgLineFillCopy(buffer,"acos");
  else if (p0[0]==KEY_EX) ProgLineFillCopy(buffer,"e^x");
  else if (p0[0]==KEY_10X) ProgLineFillCopy(buffer,"10^x");
  else if (p0[0]==KEY_LN) ProgLineFillCopy(buffer,"ln");
  else if (p0[0]==KEY_LOG) ProgLineFillCopy(buffer,"log");
  else if (p0[0]==KEY_1X) ProgLineFillCopy(buffer,"1/x");
  else if (p0[0]==KEY_ROUND) ProgLineFillCopy(buffer,"round");
  else if (p0[0]==KEY_POW) ProgLineFillCopy(buffer,"y^x");
  else if (p0[0]==KEY_SIN) ProgLineFillCopy(buffer,"sin");
  else if (p0[0]==KEY_ASIN) ProgLineFillCopy(buffer,"asin");
  else if (p0[0]==KEY_TAN) ProgLineFillCopy(buffer,"tan");
  else if (p0[0]==KEY_ATAN) ProgLineFillCopy(buffer,"atan");
  else if (p0[0]==KEY_X2) ProgLineFillCopy(buffer,"x^2");
  else if (p0[0]==KEY_SIGN) ProgLineFillCopy(buffer,"+/-");
  else
  {
    buffer[0]=0;
    for (i=0;p0[i];i++) buffer[i]=p0[i];
    buffer[i]=0;
  }
}

static void ProgLineFillCopy(unsigned char *buffer, const char *msg)
{
  int i;
  for (i=0;msg[i];i++) buffer[i]=msg[i];
  buffer[i]=0;
}


void DrawStack(bool menu, bool input, int stack_pointer)
{
  int i,j=4,k,k_end,l,m;
  if (menu) j--;
  if (input) j--;
  for (i=0;i<j;i++)
  {
    gotoxy(0,i);
    putchar('0'+j-i);
    putchar(':');
    if ((stack_pointer-j+i)>=0)
    {
      //This should be done at the end of all calculations
      //FullShrinkBCD(BCD_stack+(stack_pointer-j+i)*MATH_CELL_SIZE);
      if (BCD_stack[(stack_pointer-j+i)*MATH_CELL_SIZE+BCD_DEC]==0)
      {
        PadBCD(BCD_stack+(stack_pointer-j+i)*MATH_CELL_SIZE,1);
      }
      //CopyBCD(p1,BCD_stack+(stack_pointer-j+i)*MATH_CELL_SIZE);
      CopyBCD_EtI(p1,BCD_stack+(stack_pointer-j+i)*MATH_CELL_SIZE);

      if (Settings.SciNot)
      {
        if (IsZero_RAM(p1)) LCD_Text("0.e0");
        else
        {
          //printf("*");
          //PrintBCD(p1,-1);
          k=0;
          for (l=0;l<p1[BCD_LEN];l++) if (p1[l+3]) k=l;
          p1[BCD_LEN]=k+1;
          //printf("!");
          //PrintBCD(p1,-1);
          //printf("*");
          //getch();

          for (l=0;l<p1[BCD_LEN];l++) if (p1[l+3]!=0) break;

          m=0;
          k=(p1[BCD_DEC]-l-1);//length of e
          if (k<0) k=-k;
          if (p1[BCD_SIGN]) m++;
          if (k>9) m++;
          if (k>99) m++;
          if ((p1[BCD_DEC]-l-1)<0) m++;

          //if ((16-m)>(p1[BCD_LEN]-l))
          if (((SCREEN_WIDTH-4)-m)>(p1[BCD_LEN]-l))
          {
            k_end=p1[BCD_LEN]-l;
            m=(SCREEN_WIDTH-3)-k_end-m;
          }
          else
          {
            k_end=(SCREEN_WIDTH-5)-m;
            m=2;
          }

          gotoxy(m,i);
          if (p1[BCD_SIGN]) putchar('-');
          for (k=0;k<k_end;k++)
          {
            putchar(p1[k+l+3]+'0');
            if (k==0) putchar('.');
          }

          putchar('e');
          k=p1[BCD_DEC]-l-1;
          if (k<0)
          {
            putchar('-');
            k=-k;
          }
          m=0;
          if (k/100) {putchar('0'+(k/100));m=1;}
          if (((k%100)/10)||m) {putchar('0'+(k%100)/10);m=1;}
          putchar('0'+k%10);

          //printf("Len: %d\n",x1[BCD_LEN]-i);
          //printf("E: %d\n",x1[BCD_DEC]-i-1);
        }
      }
      else
      {
        k=p1[BCD_LEN];

        while ((p1[k+2]==0)&&(k!=p1[BCD_DEC]))
        {
          p1[BCD_LEN]-=1;
          k--;
        }
        k_end=p1[BCD_LEN];
        if (k_end>=(SCREEN_WIDTH-2))
        {
          k=0;
          k_end=(SCREEN_WIDTH-2);
          if (p1[BCD_SIGN]) k_end--;
          if (p1[BCD_DEC]<k_end) k_end--;
        }
        else if (k_end==(SCREEN_WIDTH-3))
        {
          k=1;
          if (p1[BCD_SIGN]) k=0;
          if (p1[BCD_DEC]<k_end)
          {
            if (k==0) k_end--;
            else k=0;
          }
        }
        else
        {
          k=SCREEN_WIDTH-k_end-2;
          if (p1[BCD_SIGN]) k--;
          if (p1[BCD_DEC]<p1[BCD_LEN]) k--;
        }

        gotoxy(k+2,i);
        if (p1[BCD_SIGN])
        {
          putchar('-');
          k++;
        }
        for (l=3;l<k_end+3;l++)
        {
          //if (p1[BCD_DEC]==k-3) putchar('.');
          //putchar(p1[k]+'0');
          putchar(p1[l]+'0');
          if (p1[BCD_DEC]==l-2)
          {
            if (l+k<(SCREEN_WIDTH)) putchar('.');
          }
        }
        if (p1[BCD_DEC]>k_end)
        {
          gotoxy((SCREEN_WIDTH-1),i);
          putchar('>');
        }
      }
    }
  }
}

void DrawInput(unsigned char *line, int input_ptr, int offset, bool menu)
{
  //#pragma MM_VAR line

  int i,j=4;
  bool done=false;

  if (menu) j--;
  gotoxy(0,j-1);

  SetBlink(false);
  for (i=0;i<SCREEN_WIDTH;i++)
  {
    if ((i==0)&&(offset>0)) putchar('<');
    else if ((i==SCREEN_WIDTH-1)&&(line[i+offset+1])&&(!done))
    {
      if (line[i+offset])
      {
        putchar('>');
        //gotoxy(25,6);
        //printf("%c %d     ",line[i+offset+1],i+offset+1);
      }
    }
    else
    {
      if (!done)
      {
        if (line[i+offset]) putchar(line[i+offset]);
        else
        {
          done=true;
          //putchar(' ');
        }
      }
      if (done) putchar(' ');
    }
  }
  gotoxy(input_ptr-offset,j-1);
  SetBlink(true);
}

static void ErrorMsg(const char *msg)
{
  int i,j=0,tx,char_max=5,height=1;

  enum {CUST_NW,CUST_NE,CUST_WE,CUST_SW,CUST_S,CUST_OK_O,CUST_OK_K,CUST_E};

  for (i=0;msg[i];i++)
  {
    if (msg[i]=='\n')
    {
      j=0;
      height++;
    }
    else
    {
      j++;
      if (j>char_max) char_max=j;
    }
  }
  tx=SCREEN_WIDTH/2-(char_max+1)/2;
  if (height==4) height=0;
  else height=1;
  gotoxy(tx-1,height-1);

  //putchar(CUST_NW);
  //for (j=0;j<char_max;j++) putchar('_');
  //putchar(CUST_NE);
  //gotoxy(tx-1,height);
  //putchar(CUST_WE);

  putchar('*');
  for (j=0;j<char_max;j++) putchar('*');
  putchar('*');
  gotoxy(tx-1,height);
  putchar('*');

  j=0;
  for (i=0;msg[i];i++)
  {
    if (msg[i]=='\n')
    {
      for (;j<char_max;j++) putchar(' ');
      //putchar(CUST_WE);
      putchar('*');
      j=0;
      height++;
      gotoxy(tx-1,height);
      //putchar(CUST_WE);
      putchar('*');
    }
    else
    {
      j++;
      putchar(msg[i]);
    }
  }
  /*for (;j<char_max;j++) putchar(' ');
  putchar(CUST_WE);
  height++;
  gotoxy(tx-1,height);
  putchar(CUST_SW);
  for (j=1;j<char_max;j++) putchar(CUST_S);
  putchar(CUST_OK_O);
  putchar(CUST_OK_K);*/

  for (;j<char_max;j++) putchar(' ');
  putchar('*');
  height++;
  gotoxy(tx-1,height);
  putchar('*');
  for (j=1;j<char_max;j++) putchar('*');
  putchar('O');
  putchar('K');

  while (GetKey()!=KEY_ENTER);
}

static void Number2(int num)
{
  if (num<10)
  {
    putchar(' ');
    putchar(num+'0');
  }
  else
  {
    putchar(num/10+'0');
    putchar(num%10+'0');
  }
}

void delay_ms(int ms)
{
  volatile long i,j;
  j=ms*3000;
  for (i=0;i<j;i++);
  //while (ms--) __delay_cycles(DELAY_TIME);
}

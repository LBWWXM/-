#include "string.h"//strcpy函数用
#include  "stdio.h" //sprintf函数
#include "definition.h"
#include "config.h"
#include "drivers.h"
#include "sub.h"
#include "delay.h"
#include "math.h"
#include "stdlib.h"


extern char disbuf[40];//显示缓存单元
extern u32 ad7730_filter[26];
extern u32 ad7730_code_result[26];
/*************************************************主文件变量的定义*************************************************************/

char chartemp[10][23];

volatile float setvalue=0,z1=0,z1_temp=0,z2=0,z2_temp=0,z3=0,z3_temp=0,z=0,z_temp=0,input_rate=1,sec=0,speed_v1=0,speed_code=0,
				servicepressvalue0=0,servicepressvalue1=0,tracevalue=0,pressvalue=0,speedvalue=0,avervalue=0,speedpra=0,zb=0,zd=0,zi=0,
				nload=1,tarevalue=0,realvalue=0,ctrealvalue,total_temp=0,total_cal=0,bsetend=0,ai=0,av=0,ap=0,ac=0,ao=0,setvaluetemp=0,lsreal=0, 
				ctrlana1=0,ctrlana2=0,presspra=1,danum=0,danumb=0,ek=0,ek1=0,ek2=0,ki=0,kpi=0,kp=0,kd=0,kpv=0,aq=0,ctrlout=0,ctrloutb=0,devval=0,vspeedpra=0,stoppressvalue=0,setpra=0,
				settare=0,z1clear=0,z2clear=0,z3clear=0,kjtotal=0,kj_temp=0,G=0,Xn,press=0,press_1=0,ms_secb,averpress[press_ak],real_1=0,Sepapulse=0,
				real=0,setvalue_1=0,analogpra=0,analog0=0,analog1=0,analogpra1=0,analog2=0,analogpra2=0,totalshift_temp[3]={0},avernum[20]={0},
                totalshift[3]={0},speed_dis=0,speed_v=0,speed_hz=0,lsvalue=0,lsset=0,lastday_total={0},sumquality=0,boardset=0,partmoment,angledive,
                speedcal,flowcal,boardcal,whtload,plusecount=0,speed_set,plusesum,tare_date[TARE_NUM];
volatile uint8_t warnflag=0,s2=0,s3=0,s4=0,s5=0,s6=0,s7=0,s9=0,c1=0,c2=0,c3=0,s5_flag=0,s6_flag=0,run=0,ms_sec=0,dis_dissigal_flag=0,tare_flag=0,//原s8变s9
				clearflag1=0,clearflag2=0,vmode=0,clearflag3=0,clearflag4=0,clearflag5=0,enablerun=0,mainboardflag=0,flag_int7=0,h5=0,tare_cycle_flag,
				e5=0,h1=0,h2=0,h3=0,h4=0,h6=0,h7=0,h9=0,calflg=0,e1=0,disrunflg=1,warnablerun=1,indis=1,T1_overflag=1,key_mode_flag=0,
				l1=0,l2=0,l3=0,l4=0,speedt=0,disk=0,tk=0,tk1=0,runk=0,clearflag6=0,toni=0,tonz1=0,tonz2=0,tonz3=0,ad_flag=0,Timedata=0,tareline=0,tare_flag1,
				lsbit1=0,lsbit2=0,lsbit3=0,keyflag=0,errorflag=0,pulseon=0,bctr=0,d_out=0,shift,t_tmp=0,keymode=0,tx0_count=0,tx0_sec=0,Zero_flag=0,
				dis_signal=1,lastyear=0,lastmonth=0,lastday=0,lasthour=0,lastminute=0,pre_feeder=0,cali=1,avmode=0,DS3234_status_flag=0,	
				dis_pointi=0,dis_pointz1=0,dis_pointz2=0,dis_pointz3=0,z1_point=0,z2_point=0,z3_point=0,t=0,temp_neg_flag=0,speed_count,
				ferq=0,s5_second=0,s6_second=0,hsec=0,usart_send_time=0,pulsecount=0,npulse=0,minwarn=0,dis_time_flag=0,ad7730stu=0,INT7_flag=0,
				maxwarn=0,deviout=0,ak=0,speedcount=0,speedtime=0,speedp=0,curtime=0,menui=1,seri=1,temper_flag=0,c4=0,flash_light=0,
				parai=1,dsec=0,running=0,disi=0,key_start=0,password=0,password1=0,day_locking_falg=0,check_day_flag=0,sart=0,r_flag=0,
				jgdis=0,speedless=0,Alter_time_flag=0,bflag=0,fser=0,r_temp[11]={0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0xff},
				dis_k=0,allow,Tsernum=0,flag1,flag2,flag3,rs_temp[13]={0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0xff},USART_RX_BUF[30],
				tra_temp[8]={0},pma[2]={0},sp_count=0,t_crc=0,r_crc=0,addr,ri_count=0,menu_exit_flag=0,dis_sec_count=0,runflag_count=0,
				ts_temp[63]={0},prn_time[12]={0},prnbuf1[8]={0},prnbuf2[8]={0},prnbuf3[8]={0},prnbuf4[8]={0},status[7]={0},RE_DE_Count=0,
				newpass_flag=0,in_num_flag=0,K_neg_flag=0,ready_flag=0,mainboardflagb,deputyboardflag,ctrlspeed,flashblag,modecount,separa,cycle_count,cycle_flag,time_flag,tare_count;

volatile uint16_t    analogcode=0,anainput=0,lscheck=0,rjtime=0,tempcode=0,check_value=0,ccheck_value=0,pulsewait=0,speedcode=0,speedcode_tal=0,timer_count,
					 pluscountup=0,plusmul,tarecount=0,tare_s=0,tares=0,tim_st=0;
volatile int16_t 	damin=0,damax=0,daminb=0,damaxb=0,avdanum=0,avdanumb;//转子速度码值  挡板码值调节
volatile uint32_t  	taretime=0,totalpulse=0,ontime=0,runtime=0,day_flag=0,addat=0,servicetime=0,buf7730=0,cycplussum1=0,cycss=0;
volatile u16 flow_data[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   //0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//CPU外给定输入滤波单元
volatile float xram[PRA_NUM],tare_date_result[TARE_NUM];//SRAM //0-128     
volatile float Q_quality[200],Q_lenth[200];//转子秤 
union SRAMDS3234  DS3234SRAM;//	DS3234 SPAM存变量
struct time DS3234time[2];//DS3234 时间参数值
 u8 mhang0, mhang1, mhang2;
//////////////////////////////////////////中断变量定义/////////////////////////////////////////////
u32 t_val,t_ctrl,e_val,e_ctrl,s_val,s_ctrl,e5_val,e5_ctrl;   //定时器//外部中断//速度计数器中断

					
int main(void)
{
   uint8_t i=0;
   CLI();//关中断
// delay_ms (1000);
   config();//所有资源的外设配置 IO配置 定时器 计数器配置
   d_out=1;//开关量输出状态位 1时允许输出
   dout(0,0);dout(1,0);dout(2,0);dout(3,0);dout(4,0);dout(5,0);dout(6,0);dout(7,0);dout(8,0);//开关量复位
   delay_ms (500);
   d_out=0;//关闭开关量输出
   clk_74 =0;
   delay_ms (5);
   clk_74 =1;//上升沿有效
   delay_ms(100);
   Init_LCD16032_DS3234();//显示屏初始化
   ad7730_init();//AD7730初始化
   DS3234_READSRAM_VALUE(); //读出存入存入SRAM的各种参数readpra(); //存放默认变量参数  DS3234
   check_pra();//存储参数是否正确  不正确把默认参数写入EEPROM //现在 如果下载默认参数，将导致DS3234变量被初始化 
   CLI();
   readpra();   //在读到eeprom
   readpra1();	
   CLI();	
   display_check();//显示检查 如果输入key_input键，输入相应的数据会装入默认参数  输入锁机开始时间
   CLI();
   calc_dat();	//系数计算
   if(z1<0){z1=0;z1_temp=0;}//z1总累计量 z1_temp 100ms累计量
   if(totalshift[0]<=0)totalshift[0]=0;//Z1//班组累计
   if(totalshift[1]<=0)totalshift[1]=0;//Z2
   if(totalshift[2]<=0)totalshift[2]=0;//Z3累计量 存储于DS3234中（12C887）
   run=running=0;//运行位 
   for(i=0;i<press_ak;i++)averpress[i]=0;//press_ak=20 //压力平均缓存单元
   press=press_1=0;
   sumquality=addat*presspra-tarevalue;//重量 kg  //没有用 总中断被关
   dissignal();//菜单画面显示保持界面函数
   d_out=1;//后加
   if(!keymode)//非键盘模式  keymode =1 键盘模式  keymode =0 停键盘模式 备妥输出在定时器里也有，
   {
	    dout(7,0);//秤停止
		  dout(8,1);//备妥信号输出  程序已经开始执行，证明
      if((xram[43]==1)&&(xram[46]==1))//P43串行通讯地址选择1有串行通讯，0无串行通讯。//启动设定值的来源都来自串口 
		  status[0]=0x31;//有备妥  //串口发送用
 		  else //启动为开关量x3:3,4  设定值由模拟量x6:1,2输入或是由键盘给定
		  status[0]=0x30;//无备妥
   }
    else//键盘模式  启动和设定值均有键盘设定
   {
	    dout(8,0); //键盘模式 不需要输出备妥
		  status[0]=0x30; //串口发送用 （无备妥）
   }
   delay_ms(1);
////d_out=1;//开关量允许输出 //有问题 待改
   SEI();

 while(1)
{ 
  delay_us(3);
	check_day();//检查锁机时间 //锁机时间到day_locking_falg=1 check_day_flag=1; xram[125]=1;下载默认参数 读参数   有写flash
	delay_us(3);
	recive_key();
	delay_us(3);
	if(((jgdis>1)||keyflag)){dissignal();jgdis=0;}//键盘按下没松开键盘 标志位等于1  表明按下按键显示
	else jgdis++;

	/*****************************流量报警处理***********************************///(指示灯报警 数字量输出)
//     if(din(0))clear_warn();//开关量输入  故障应答 DI1 //此清除警告函数无效   没用 没用
	if(maxwarn){dout(1,1);dout(11,1);} //最大值数字量输出报警，最大值指示灯报警  DO2
	else {dout(1,0);dout(11,0);}		   
	if(minwarn){dout(3,1);dout(12,1);}//最小值数字量输出报警，最小值指示灯报警   DO3           
	else {dout(3,0);dout(12,0);}//其他情况，最小值输出报警关，最小值指示灯报警关

	/********************************事件信息检测*************************************/
    ////////跑偏开关处理/////// 跑偏后仪表秤的停止  皮带秤用，转子秤可能不需要
	if(din(2)) h9=0;//DI3=0 可接跑偏开关//没有跑偏//释放    //皮带跑偏或现场控制       输入为0   （开关闭合时） //DI3
	else//DI3=1//有跑偏  （开关断开时）                                         跑偏现场确定  
	{
		h9=1;
		if(xram[41]==1)//1 跑偏开关断开时，仪表控制器电流环停止//0 跑偏开关断开时，仪表电流环不停止
		{
// 			r_temp[9]=0x30;//没有用到此数组
			stop();//run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0; 
		}
	}
	////////转子秤负载事件 ////////  P10荷重传感器负载的总容量  默认值60公斤
	if((sumquality+tarevalue)>(xram[10]*1.1)){h4=1;l4=0;}//荷重传感器负载*1.1/有效称量段长度//H4 称重系统过载//超过称重传感器荷重负载的1.1倍  66公斤
	else h4=0;
	
	
	if((sumquality+tarevalue)<(xram[10]*0.03))//荷重传感器负载  1.8公斤
	{
		
			l4=1;//称重系统负载过小
			h4=0;
		
	}
	else l4=0;
	 ////////转子秤流量事件 //////// kg/h
	if((tracevalue>xram[21])&&(running>5))//流量上限 计算 确定仪表确实是启动了//P21流量上限 启动时间大于5S   启动时间待改
	{
		if(tracevalue>xram[21])//流量超过上限   
		{
			h1=1;l1=0;//实际流量超过上限值
			maxwarn=1;//开关量上限输出
			minwarn=0;
		}
	}
	else
	{
		maxwarn=0;
		h1=0;
	}
	if((tracevalue<=xram[22])&&(running>5))//流量下限 tracevalue是实际平均流量值//P22流量下限 启动时间大于5S
	{
		if(tracevalue<=xram[22])//流量下于等于下限
		{
			l1=1;h1=0;//实际流量低于下限值
			minwarn=1;//流量下限报警
			maxwarn=0;
		}
	}
	else
	{
		minwarn=0;
		l1=0;
	}
	
	////////转子秤荷重事件 ////////  kg
	if((whtload>xram[23])&&(running>5))//荷重上限 //滤波后的荷重平均值（20） //P23荷重上限  启动时间大于5S   上限200
	{
		if(whtload>xram[23])//负荷超出上限 
		{
			h2=1;l2=0;
		}
	}
	else h2=0;
	if((whtload<xram[24])&&(running>5))//荷重下限 //滤波后的荷重平均值（20） //P24荷重下限  启动时间大于5S   下限5
	{
		if(whtload<xram[24])
		{
			l2=1;//负荷低于下限 
			h2=0;
		}
	}
	else    l2=0;
	////////流量偏差事件检测及处理////////
	if((fabs(setvalue-tracevalue)>xram[37])&&(running>5)){h5=1;dout(4,1);}//流量偏差范围超出 //P37流量偏差范围 偏差输出  设定值减反馈
	else {h5=0;dout(4,0);}//偏差输出关
	////////流量设定值事件检测////////
	if((!avmode)&&(setvalue<(xram[1]*0.02)))e5=1;//在非容积模式：//设定值低于低限  P1额定给料量
	else e5=0;
	////////传感器输入错误事件检测//////// （待改）
	if((addat<30||(addat>65532))&&(running>5))c1=1;//传感器输入出错  //24位 用了16位 
	else  c1=0;//正常
	////////电机故障////////if((speedcode<3)&&xram[9])speedless++;else speedless=0;//速度传感器无效//速度传感器有效  P9速度传感器是否为有效。 0速度传感器无效，速度由P5决定。1速度由传感器输入	            	
	if((speedless>10)&&run&&(ctrlout>2000))  h7=1;//连续10s不正常时//speedcode小于3 超多10s  //速度给定2000
	else  h7=0;	                               //待改
	////////速度传感器输入错误////////
	if((speedcode>1500)&&xram[9]&&(running>5))  //待改
	{
		if((speedcode>1500)&&xram[9])c3=1;//75*20 速度传感器输入错误
		else c3=0;
	}	
	////////速度输出控制事件///////
	if(((ctrlout/speedcal)>=xram[31])&&run)h6=1;//控制输出最大值 //控制器到上限  根据公式：20.8845*B/4096 =输出电流  待改
	else h6=0;
	////////流量设定值输入错误///////
	if(((setvaluetemp>xram[1]*3)&&avmode)||(!avmode&&(setvaluetemp>xram[1])))s9=1;//设定值超出极限//流量上限
	else s9=0;
	////////速度传感器检测///////
// 	if((speed_check==0)&&xram[9])c2=1;//读取速度检测端口，等于0 速度报警   //速度传感器输入出错
// 	else c2=0;
	////////预给料机的启动停止///////  待改 现场交流
	if(pre_feeder&&(run))dout(5,1);//预给料机启动   预给料标志1    
	else dout(5,0);//预给料机关闭	
	
/**********************检测变量*******************/	
	if((dis_signal<1)||(dis_signal>10))dis_signal=2;//下行显示变量
	if((pre_feeder<0)||(pre_feeder>1))pre_feeder=0;
	if((keymode<0)||(keymode>1))keymode=0;//键盘模式标记位
	if((avmode<0)||(avmode>1))avmode=0;
	if(jgdis<0)jgdis=0;    
 }		
}	  
/**********************速度计数中断*******************/	//转子速度控制   //只有运行时，称重信号才能更新
void TIM8_UP_IRQHandler  (void)   //TIM8中断  转子速度的控制周期
{
 if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)  //检查TIM8更新中断发生与否
 {  
	   u16 i,j,avt;//
	   u32 addata=0,count,a=0;
	   TIM_ClearITPendingBit(TIM8, TIM_IT_Update);  //清除TIMx更新中断标志 //注意此程序  不清除不能进入中断   定时器计数器一直在计数
// 	 TIM_SetCounter(TIM8, 0);//计数器清零
	 /////////////SysTick延时寄存器数据备份//////// 
	   s_val=SysTick->VAL;//不为0 则 16位 为0 为0 16位清零
	   if(s_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器1
	   s_ctrl=SysTick->CTRL;
// 	 cpu_led=~cpu_led;
	   plusmul++;  //和速度定时器
	   avt=tare_s; //皮重的对应值
     tare_s++;
  /**************************** AD7730码值处理*********************************///用了16位精度
//if(Zero_flag==1)
//{
    ad_flag=1;//控制AD中断的读取
		for(i=0;i<26;i++)
		{
		  ad7730_code_result[i]=ad7730_filter[i];  //数据转移
		}
		qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//对数据排序  从小到大排序
		addata=0;
		for(i=5;i<21;i++)
		{
			addata+=ad7730_code_result[i];//5到20相加  16个数 
		}
		addata>>=12;//左移12位，除以4096,？左移4位，除以16，左移8位除以256//去了8位精度 //除以16，求取码值的平均值  24位去了8位精度 16位
		addat=addata;//16位码值   //100ms更新一次
		ad_flag=0; //控制AD中断的读取
	  sumquality=addat*presspra-tare_date_result[avt];
		
//}	

	 //皮带负荷  减去皮重    //需要修改  kg （现场是否需要滤波处理）	 码值（用质量代表力）  代表着力   皮重代表着力
	 /******************************运行模式转变**************************/ 
	  plusecount++;//单位旋转脉冲计数
// if(tare_flag1==1&&tareline==1)    //除皮标定，标定圈脉冲数 //进入中断后判断是否从零点开始称重
// 		{ 
// 			j=tarecount;  //皮重次数
//      	a=addat;      
// 			tare_date[j]+=a;			 //将码值加到数组里面对应的第j位
// 	    tarecount++;           //数组次数+1
//    }
	  if(modecount==1) //清料  挡板关 转子按设定转速旋转
    { 
		 EXTI_Set_Int(1);
     plusesum+=plusecount*xram[63];//运行模式1，记录旋转脉冲，把积累的料转出去  xram[63]分隔脉冲数
		 if((plusesum>=2*xram[64])&&Zero_flag==1) //累计角度大于有效角度//旋转总脉冲数大于有效角度脉冲数  
		 {
			plusesum=0;  //旋转脉冲计数
			modecount=2;	//模式计数
		  plusecount=0;//计数中断的计量
			EXTI_Set_Int(0);
     }
    }
	   else 
		 if(modecount==2) //模式2时间到转换到模式3  //挡板开下料    
		 {
			 plusesum+=plusecount*Sepapulse;//记录总累计脉冲
			 if(plusesum>=2*xram[64]) //旋转总脉冲数大于有效角度脉冲数
			 {
				  plusesum=0;//累计脉冲数清零
				  modecount=3;//模式3
			    plusecount=0;//计数中断的计量
			 }
		 }
	   /**********控制参数求取（q1用于求取流入流量和挡板控制//qn求取反馈流量和控制速度）*************/
		if(modecount==2||modecount==3)
		 {    
			   
				 partmoment=0;
				 for(count=separa;count>1;count--) //1到seprara 
				 {  
					 Q_quality[count] =Q_quality[count-1];//数组右移	  分隔旋转
				 } 				 
				 for(count=2;count<=separa;count++)//计算力矩
				 {
					 partmoment+=Q_quality[count]*Q_lenth[count];  //长度单位CM
				 }
				 Q_quality[1]=(sumquality*xram[70]-partmoment)/Q_lenth[1];//求出q1  用作挡板控制  P70: L  动力臂*动力=阻力臂*阻力臂  有可能出现负数。由于计量的不准确性
				 whtload =0;
				 for(count=1;count<=separa;count++) //只有运行的时候 才有负荷  不运行时，改显示那个负荷  
				 {
              whtload+=Q_quality[count]; //计算总负荷
         }
				 if(modecount==3) //正常模式3   用以下公式
				 {	 
				   if(Q_quality[separa]>0)
				  {
				   speed_set=(setvalue*angledive)/(Q_quality[separa]*3.6);	//求得给定速度  setvalue=Q_quality[separa]/angledive*speed_set*3.6 =kg/°*°/s*3.6
				  } 
				 }  
			}
		 
	     /*******************************流量的计算处理*******************************************/  //流量均值的求取
	  if(modecount==1)  //刚启动时，控制模式1  清料的过程
		  {
// 			 press=sumquality/(xram[64]-anglemode);//模式1的流量控制   模式1剩余角度
			   press=0;//计算流量用
			   Xn=0;//计算流量用
		  }
		else if((modecount==2)||(modecount==3)) //2 3
		  {
			 	 press=Q_quality[separa]/angledive;//单位度数的重量  kg/度   //流量反馈用    //每个扇形的角度angledive
			   Xn=Q_quality[1]/angledive;// kg/度   挡板控制用   (分隔角度)  //（63 分隔角度脉冲数）/（77 一圈的总脉冲数）*360
		  }
			press=press_1+aq*(press-press_1);//AQ荷重滤波  //滤波后的荷重  //
			press_1=press;
			for(i=1;i<press_ak;i++)averpress[i-1]=averpress[i];//press_ak=20  数组向左移动 0-press_ak-1   20个 到19
			averpress[press_ak-1]=press;
			pressvalue=0;  //
			for(i=0;i<press_ak;i++)pressvalue+=averpress[i];
			pressvalue/=press_ak;//滤波后的荷重取均值   20个均值  //显示流量的计算用到此变量  
			
		/************************************速度的控制***************************************/ 
  if((run)&&(!calflg))//运行并且非标定模式  
	{
			////////////////////驱动和应答////////////////////
		 
			if(modecount==1)//刚启动控制模式1 关挡板 开转子速度 （剩余的料漏下去）
			{
			    ctrlout=xram[39]*speedcal;//P39容积方式驱动电流 4096/22.9  最大码值对应的最大电流 //待改  （以容积方式控制转速�
      }
			else if(modecount==2)  //开挡板和转子速度  分隔单元开始记分隔单元的重量
			{   
          ctrlout=xram[39]*speedcal;//P39容积方式驱动电流 4096/20.88  最大码值对应的最大电流 //待改  （以容积方式控制转速）         				
      }
      else if(modecount==3)  //容积方式（手动模式）挡板的控制  速度的控制放在计数器里
			{   
				if(avmode||(vmode))//容积方式。在重量工作当中，启动开始时，先按以下控制一段时间
				{  
				  ctrlout=xram[39]*speedcal;//P39容积方式驱动电流 4096/22.9  最大码值对应的最大电流 //待改   
					ctrlout+=avdanum;
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //容积方式转子转速的控制
			  }
// 		  else if((xram[38]==1)&&(h5==0)){}//偏差范围内不调速P38偏差控制选择：当选择0时，在任何时间内调节点击速度。当选择1时，在偏差范围内不调节电机速度  在偏差不超限时
				else//重量工作方式                               //选择此参数可减小电机震荡，使得物料运行平稳。 
				{
				  if((bflag==0)||(((zb-zi)>0)&&bflag))//批量生产标志位0 bsetend 5*set_value/3600//流量设定值  ((zb-zi)>bsetend)&&bflag)  zb批量生产设定值 zi批量生产增量
				  {   
					ctrlout=speed_set/xram[69]*analogpra1+analog1; //xram[68]=0//速度对应码值/速度=单位速度码值
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //容积方式转子转速的控制
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //批量生产结尾控制  去掉
					{
						if(bctr==0)
						{
							danum=ctrlout/220;
							bctr=1;
						}
						ctrlout-=danum;//100ms的控制周期  缓慢的减小
						if(ctrlout<(damin+300)) ctrlout=damin+300;
						else if(ctrlout>damax) ctrlout=damax;
						if(ctrlout>4095) ctrlout=4095;
					}
				}
		 }
  }
		if(mainboardflag==0)//当主板流量输出校准时，此标志位置1 不可以输出
		{
		   DAC7612(0,ctrlout);
		}//速度控制输出       这里会加上一个挡板
 /////////////SysTick延时寄存器数据还原////////
			SysTick->LOAD=s_val;
			SysTick->VAL=0x00;
			SysTick->CTRL=s_ctrl;
	 
  }

}

/*********************************定时器3比较中断********************************/

//定时器3中断服务程序
void TIM3_IRQHandler (void)//TIM3中断
{
 if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
 {            
	  float speed_value=0;
	  uint16_t i,DS3234_code;
	  uint32_t ls_sec,addatab;
	  TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 //注意此程序  不清除不能进入中断   定时器计数器一直在计数
	  /////////////SysTick延时寄存器数据备份//////// 
		t_val=SysTick->VAL;//不为0 则 16位 为0 为0 16位清零
		if(t_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器1
		t_ctrl=SysTick->CTRL;
	   ///////////////////////////////////////////
		t_tmp++;//*
		flash_light=!flash_light; //取反指令//故障报警指示灯用
	   ///////////////////////////////////////////
	  if(run==0&&flashblag==1)  //计算参数在改变参数时加 在秤运行时 修改参数时执行
	  {
        writepra();
	     flashblag=0;  
    }
    /////////////////////////////////速度计算///////////////////////////////////	
		if(xram[9])//速度传感器是否有效？1，有效，否侧无效。无效时，由额定速度确定
		{
			speedcode=plusmul*Sepapulse+TIM_GetCounter(TIM8)-pluscountup;//计数器脉冲数  P77:分隔脉冲数
			pluscountup=TIM_GetCounter(TIM8);//记录本次定时的脉冲值
			plusmul=0;//倍数清零
			speed_count++;//定时器中断次数
			speedcode_tal+=speedcode;//计数相加 //转子秤speedpra（单个脉冲转子旋转的角度）
// 			anglecount =speedpra*speedcode;//记录旋转角度 100ms转子秤转了多少角度
			speed_value=10*speedpra*speedcode;//计算速度。speedpra单个脉冲对应皮带所走长度  为什么乘以10   每100ms皮带走了x ,1s走了10*x =speed_value 1S皮带行走的长度
			speedvalue=0.6*speedvalue+0.4*speed_value;//加权 speedvalue流量计算时用 
			if(speed_count==10)//10个数平均  1s
			{
				speed_code=speedcode_tal/10.0;
				speed_v=10*speedpra*speed_code;//采样10次的平均速度  M/s
				speed_dis=speed_v1+av*(speed_v-speed_v1);//平均速度滤波
				if(speed_dis<0)speed_dis=0; 
				speed_v1=speed_dis;//速度值 //不清零  put_disdat函数调用  //在dissignal(void)调用 主显示界面 speed_v1没有用
				speed_hz=speedcode_tal;//总脉冲数  定时器中断10次的总脉冲数put_disdat函数调用  速度频率  1s计数器计数值 
				speedcode_tal=0;
				speed_count=0;
			}
		}
		else{speedvalue=xram[5];}//*/ //额定速度  m/s  0.1  反馈量用额定速度
		
	
// 		if(Timedata>=2) //300ms初始化AD7730  //如何理解这个初始化     //待改
// 		{
// 			c4=1;//C4AD7730采样错误
// 			EXTI->IMR &= ~(EXTI_Line15);// 屏蔽外部中断15;//关闭外部中断
// 			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//串口3接收使能禁止
// 			
// 			AD_RST =0;
// 			delay_us(40);
// 			AD_RST =1;//1
// 			delay_us(40);
// 			Timedata=0;
// 			ad7730_init();
// 			EXTI->IMR |= EXTI_Line15;//使能外部中断15;//开外部中断 AD7730中断
// 			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//串口3接收使能禁止//串口3接收中断使能开
// 		}
		Timedata++;//*/ AD7730初始化用  每200ms初始化一次
		DS3234_WRITESRAM_VALUE();//部分定义变量向sram里写
		/*****************************读取当前时间*********************************/
		DS3234time[0].second=Read_data_DS3234(0x00); //second  0  
		DS3234time[0].minute=Read_data_DS3234(0x01); //minute
		DS3234time[0].hour=Read_data_DS3234(0x02);   //hour
		DS3234time[0].week=Read_data_DS3234(0x03);   //week
		DS3234time[0].date=Read_data_DS3234(0x04);   //date
		DS3234time[0].month=Read_data_DS3234(0x05);  //month
		DS3234time[0].year=Read_data_DS3234(0x06);   //year//*/
		/*****************************读温度码值*********************************/
		if(temper_flag==1)  //标志位在服务函数里调用  只有为1才读取温度  在服务值里用到
		{
			DS3234_status_flag=Read_data_DS3234(0x0f);//BSY bit status
			if((DS3234_status_flag&0x04)==0x04){delay_us(3);}
			else
			{
				DS3234_code=Read_data_DS3234(0x11);//高字节
				DS3234_code<<=8;
				DS3234_code+=Read_data_DS3234(0x12);//低字节  
				DS3234_code>>=6;//还有10位，最高位负号位
				delay_us(1);
				if(DS3234_code<512)//2的9次幂 为正 511
				{
					tempcode=DS3234_code;
					tempcode>>=1;//减小1位精度  /?
					temp_neg_flag=0;
				}
				else//为负
				{
					DS3234_code=(~DS3234_code);//去反 
					DS3234_code&=0x7f;
					DS3234_code+=1;//非负数 去反加1
					tempcode=DS3234_code;
					tempcode>>=1;
					disbuf[31]='-';
					temp_neg_flag=1;//正负标志位
				}
			}
		}//*/
		/*****************************时间修改*********************************/
		if(Alter_time_flag==1) //标定时间
		{
			Write_data_DS3234(0x80,DS3234time[1].second); //second
			Write_data_DS3234(0x81,DS3234time[1].minute); //minute
			Write_data_DS3234(0x82,DS3234time[1].hour);   //hour
			Write_data_DS3234(0x83,DS3234time[1].week);   //week
			Write_data_DS3234(0x84,DS3234time[1].date);   //date
			Write_data_DS3234(0x85,DS3234time[1].month);  //month
			Write_data_DS3234(0x86,DS3234time[1].year);   //year
			Alter_time_flag=0;
		}
	   /*****************************为什么？*********************************/  //在原程序里实现中断嵌套
// 	     USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//串口3接收使能//串口1接收中断使能开
// 	     EXTI->IMR |= EXTI_Line15;//使能外部中断15;//开外部中断 AD7730中断
// 	     SEI();//打开全部中断
	   /*****************************时间记录标志位*********************************/
		 if(ms_secb>=10){ms_secb=0;} //除皮标定计时  改
	     ms_secb+=1; //每100ms+1   2s时清零 
	   /**************************** 刷新显示*********************************/
		 dis_play();
           /**************************** AD7730码值处理*********************************///用了16位精度
		if(run==0)  //转子秤没有运行时，需要数据显示负荷，没有运行时的负荷量为（直接测量-皮重）没有通过杠杆比的转化
		{		
			ad_flag=1;//控制AD中断的读取
			for(i=0;i<26;i++)
			{
				ad7730_code_result[i]=ad7730_filter[i];  //数据转移
			}
			qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//对数据排序  从小到大排序
			addatab=0;
			for(i=5;i<21;i++)
			{
				addatab+=ad7730_code_result[i];//5到20相加  16个数 
			}
			addatab>>=12;//左移12位，除以4096,？左移4位，除以16，左移8位除以256//去了8位精度 //除以16，求取码值的平均值
			addat=addatab;//16位码值 
			ad_flag=0; //控制AD中断的读取
			sumquality=addat*presspra-tarevalue;//sumqualit 为称重传感器直接测量的没有通过杠杆比的转化
	  }
		/**************************** 检查锁机时间是否已到,如果到了就退出*********************************/
		if(xram[125]==1)
	  {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志 不清除标志位，进不了中断
		 return;//检查锁机时间置位  重要 重要 
    }			
		/**************************** 计时标志位*********************************/
		if(hsec>=10)//1.0s   1s循环一次
		{
			
			hsec=0;
			ms_secb=0; 
			tx0_sec=0;//dissignal用   
			sec=sec+1;//和标定程序有关
			if(rjtime)rjtime--;//刚开始启动时，容积工作方式
			else{vmode=0;}//      
			if(run)//检测速度传感器用
			{
				if(running<200)running++; //1s检查一次   （事件信息用）
				if((speedcode<3)&&xram[9])speedless++;//速度传感器有效  P9速度传感器是否为有效。 0速度传感器无效，速度由P5决定。1速度由传感器输入
				else speedless=0;//(速度传感器无效)或是(速度传感器有效并且speedcode>3)
			}
			else {speedless=0;running=0;}//秤没有运行
			if(s5_flag);//密码有效标志位  
			else if(s5)  //普通密码
			{
				s5_second--;
				if(!s5_second)s5=0;
			}
			if(s6_flag);
			else if(s6)  //超级密码
			{
				s6_second--;
				if(!s6_second)s6=0;
			}
		}
		/*********************************串口1定时发送数据*****************************/
		if(usart_send_time>=50)
		{
			usart_send_time=0;
			if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			{
		   switch(tx0_count)//发送计数器  //发送 时间  1班累计 2班累计 3班累计 总累计
			 {
				case 0:USART_SendData(USART1, prn_time[0]);tx0_count++;break;//打印时间  年     
				case 1:USART_SendData(USART1, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART1, '-');tx0_count++;break;
				case 3:USART_SendData(USART1, prn_time[2]);tx0_count++;break;          //月
				case 4:USART_SendData(USART1, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART1, '-');tx0_count++;break;	
				case 6:USART_SendData(USART1, prn_time[4]);tx0_count++;break;          //日
				case 7:USART_SendData(USART1, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART1, 0x0d);tx0_count++;break; //回车
				case 9:USART_SendData(USART1, 0x0a);tx0_count++;break; //换行
				case 10:USART_SendData(USART1, prn_time[6]);tx0_count++;break;         //时
				case 11:USART_SendData(USART1, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART1, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART1, prn_time[8]);tx0_count++;break;         //分
				case 14:USART_SendData(USART1, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART1, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART1, prn_time[10]);tx0_count++;break;        //秒
				case 17:USART_SendData(USART1, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART1, 0x0d);tx0_count++;break;//回车 Enter
				case 19:USART_SendData(USART1, 0X0a);tx0_count++;break;//换行
				case 20:USART_SendData(USART1, 0X3d);tx0_count++;break;//=
				case 21:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 22:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 23:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 24:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 25:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 26:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 27:USART_SendData(USART1, 0X3d);tx0_count++;break;
				case 28:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 29:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 30:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 31:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 32:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 33:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 34:USART_SendData(USART1, 0X3d);;tx0_count++;break;
				case 35:USART_SendData(USART1, 0X3d);;tx0_count++;break;//=
				case 36:USART_SendData(USART1, 'T');;tx0_count++;break; //TOTAL:113213132t
				case 37:USART_SendData(USART1, 'O');;tx0_count++;break; //SHIFT:1
				case 38:USART_SendData(USART1, 'T');;tx0_count++;break;
				case 39:USART_SendData(USART1, 'A');;tx0_count++;break;
				case 40:USART_SendData(USART1, 'L');;tx0_count++;break;
				case 41:USART_SendData(USART1, ':');;tx0_count++;break;
				case 42:USART_SendData(USART1, prnbuf1[7]);tx0_count++;break;//prnbuf1  1班组累计量
				case 43:USART_SendData(USART1, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART1, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART1, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART1, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART1, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART1, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART1, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART1, 0x0d);tx0_count++;break;//回车换行
				case 52:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART1, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART1, 'H');tx0_count++;break;
				case 55:USART_SendData(USART1, 'I');tx0_count++;break;
				case 56:USART_SendData(USART1, 'F');tx0_count++;break;
				case 57:USART_SendData(USART1, 'T');tx0_count++;break;
				case 58:USART_SendData(USART1, ':');tx0_count++;break;
				case 59:USART_SendData(USART1, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART1, 0x0d);tx0_count++;break;//回车
				case 61:USART_SendData(USART1, 0x0a);tx0_count++;break;//换行
				case 62:USART_SendData(USART1, 0x2d);tx0_count++;break;//-
				case 63:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 64:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 65:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 66:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 67:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 68:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 69:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 70:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 71:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 72:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 73:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 74:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 75:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 76:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 77:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 78:USART_SendData(USART1, 'T');tx0_count++;break;//TOTAL
				case 79:USART_SendData(USART1, 'O');tx0_count++;break;
				case 80:USART_SendData(USART1, 'T');tx0_count++;break;
				case 81:USART_SendData(USART1, 'A');tx0_count++;break;
				case 82:USART_SendData(USART1, 'L');tx0_count++;break;
				case 83:USART_SendData(USART1, ':');tx0_count++;break;
				case 84:USART_SendData(USART1, prnbuf2[7]);tx0_count++;break;//prnbuf2  班组2
				case 85:USART_SendData(USART1, prnbuf2[6]);tx0_count++;break;
				case 86:USART_SendData(USART1, prnbuf2[5]);tx0_count++;break;
				case 87:USART_SendData(USART1, prnbuf2[4]);tx0_count++;break;
				case 88:USART_SendData(USART1, prnbuf2[3]);tx0_count++;break;
				case 89:USART_SendData(USART1, prnbuf2[2]);tx0_count++;break;
				case 90:USART_SendData(USART1, prnbuf2[1]);tx0_count++;break;
				case 91:USART_SendData(USART1, prnbuf2[0]);tx0_count++;break;
				case 92:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 93:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 94:USART_SendData(USART1, 0x0a);tx0_count++;break;			
				case 95:USART_SendData(USART1, 'S');tx0_count++;break;
				case 96:USART_SendData(USART1, 'H');tx0_count++;break;
				case 97:USART_SendData(USART1, 'I');tx0_count++;break;
				case 98:USART_SendData(USART1, 'F');tx0_count++;break;
				case 99:USART_SendData(USART1, 'T');tx0_count++;break;
				case 100:USART_SendData(USART1,':');tx0_count++;break;
				case 101:USART_SendData(USART1, 0x32);tx0_count++;break;
				case 102:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 103:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 104:USART_SendData(USART1, 0x2d);tx0_count++;break;//-
				case 105:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 106:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 107:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 108:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 109:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 110:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 111:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 112:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 113:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 114:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 115:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 116:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 117:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 118:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 119:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 120:USART_SendData(USART1, 'T');tx0_count++;break;
				case 121:USART_SendData(USART1, 'O');tx0_count++;break;
				case 122:USART_SendData(USART1, 'T');tx0_count++;break;
				case 123:USART_SendData(USART1, 'A');tx0_count++;break;
				case 124:USART_SendData(USART1, 'L');tx0_count++;break;
				case 125:USART_SendData(USART1, ':');tx0_count++;break;
				case 126:USART_SendData(USART1, prnbuf3[7]);tx0_count++;break;//prnbuf3  班组3
				case 127:USART_SendData(USART1, prnbuf3[6]);tx0_count++;break;
				case 128:USART_SendData(USART1, prnbuf3[5]);tx0_count++;break;
				case 129:USART_SendData(USART1, prnbuf3[4]);tx0_count++;break;
				case 130:USART_SendData(USART1, prnbuf3[3]);tx0_count++;break;
				case 131:USART_SendData(USART1, prnbuf3[2]);tx0_count++;break;
				case 132:USART_SendData(USART1, prnbuf3[1]);tx0_count++;break;
				case 133:USART_SendData(USART1, prnbuf3[0]);tx0_count++;break;
				case 134:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 135:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 136:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 137:USART_SendData(USART1, 'S');tx0_count++;break;
				case 138:USART_SendData(USART1, 'H');tx0_count++;break;
				case 139:USART_SendData(USART1, 'I');tx0_count++;break;
				case 140:USART_SendData(USART1, 'F');tx0_count++;break;
				case 141:USART_SendData(USART1, 'T');tx0_count++;break;
				case 142:USART_SendData(USART1, ':');tx0_count++;break;//3
				case 143:USART_SendData(USART1, '3');tx0_count++;break;//3
			  case 144:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 145:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 146:USART_SendData(USART1, 0x2d);tx0_count++;break;//-
				case 147:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 148:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 149:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 150:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 151:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 152:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 153:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 154:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 155:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 156:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 157:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 158:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 159:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 160:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 161:USART_SendData(USART1, 0x2d);tx0_count++;break;
				case 162:USART_SendData(USART1, 'S');tx0_count++;break;
				case 163:USART_SendData(USART1, 'U');tx0_count++;break;
				case 164:USART_SendData(USART1, 'M');tx0_count++;break;
				case 165:USART_SendData(USART1, ':');tx0_count++;break;
				case 166:USART_SendData(USART1, ' ');tx0_count++;break;
				case 167:USART_SendData(USART1, ' ');tx0_count++;break;
				case 168:USART_SendData(USART1, prnbuf4[7]);tx0_count++;break;//prnbuf4 总的累计量 
				case 169:USART_SendData(USART1, prnbuf4[6]);tx0_count++;break;
				case 170:USART_SendData(USART1, prnbuf4[5]);tx0_count++;break;
				case 171:USART_SendData(USART1, prnbuf4[4]);tx0_count++;break;
				case 172:USART_SendData(USART1, prnbuf4[3]);tx0_count++;break;
				case 173:USART_SendData(USART1, prnbuf4[2]);tx0_count++;break;
				case 174:USART_SendData(USART1, prnbuf4[1]);tx0_count++;break;
				case 175:USART_SendData(USART1, prnbuf4[0]);tx0_count++;break;
				case 176:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 177:USART_SendData(USART1, 0x3d);tx0_count++;break;//=
				case 178:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 179:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 180:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 181:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 182:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 183:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 184:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 185:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 186:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 187:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 188:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 189:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 190:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 191:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 192:USART_SendData(USART1, 0x3d);tx0_count++;break;
				case 193:USART_SendData(USART1, 0x0d);tx0_count++;break;//回车 Enter	
				case 194:USART_SendData(USART1, 0x0a);tx0_count++;break;//换行
				case 195:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 196:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 197:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 198:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 199:tx0_count=0;break;
			}
		}
	}
		 /**************************** 特殊情况的显示（事件报警显示）*********************************/
		  if(t_tmp>=1) //++在定时中断开始   每100ms执行一次
		  {
			t_tmp=0; //++在定时中断开始
			tx0_sec++;//
			hsec++;
			usart_send_time++;
			if(disrunflg)//很多函数包含此标志位
			{
				/////////////////容积键盘模式图标显示///////////////////
				if((avmode==1)&&(keymode==1))//键盘和容积模式 
				{ 
					disbuf[0]=0xa1;
					disbuf[1]=0xf8;
				}
				else if((avmode==0)&&(keymode==1))//键盘模式
				{
					disbuf[0]=0xa1;
					disbuf[1]=0xf7;
				}
				if(keymode==0)//
				{
					disbuf[0]=0x20;//不显示
					disbuf[1]=0x20;
				}	
				
				ls_sec=(uint32_t)sec;
				i=ls_sec%2;//奇数秒
				
				/////////////////批量生产图标显示///////////////////
				if(run&&i)//在运行状态且sec位奇数时
				{
					if(bflag==0) //没有批量生产
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xfa;
					}
					else //批量生产
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xc6;
					}
				}
				else//sec为偶数 或是没有运行时
				{
					if(run&&bflag)//运行且批量生产标志位1
					{
						disbuf[2]=0x20;
						disbuf[3]=0x20;
					}
					else if(bflag==0)
					{
						disbuf[2]=0x20;
						disbuf[3]=0x20;
					}
					else	
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xc6;
					}
				}
				
			  /////////////////非容积图标显示（重量工作方式刚启动）///////////////////
				if(run&&i&&vmode)
				{
					if(rjtime&&(!avmode))
					{
						disbuf[0]=0xa1;
						disbuf[1]=0xf8;
					}
				}
				else if(vmode&&rjtime&&!avmode)
				{
					disbuf[0]=0x20;
					disbuf[1]=0x20;
				}
				
				/////////////////报警事件图标显示///////////////////
				if(warnflag&&!i)//报警标志位1  闪烁用
				{
					disbuf[20]=0x20;
					disbuf[21]=0x20;
				}
				else
					warn();//事件报警显示
			  }
		   }
		 /**************************** 累计脉冲输出*********************************/
		 if(pulsecount)
		  pulsecount--;  //输出脉冲的时间 //时间计算有问题pulsecount 
 	    else
			{
				dout(6,0);   //累计脉冲输出
			}
	    /**************************** 报警事件处理*********************************/

      if(c1||h7||h4||l4||((xram[41]==1)&&h9))//C1称重传感器输入错误//h7电机故障//h4称重传感器负载过大//l4称重传感器负载太小 待改  出现故障时不输出备妥 plc秤启动信号关闭 
			{
				ready_flag=1;//备妥输出标记  //下边用      //P41 皮带跑偏控制选择 1 端子X6：5，6断开时，仪表控制器电流环停止//h9皮带跑偏或现场控制 
				if(flash_light==1)dout(10,1);//故障指示灯报警闪烁
				else dout(10,0);
				dout(2,1);//输出报警信号 DO3报警输出 继电器输出报警
			}
			else//上述事件都没发生时，
			{
				ready_flag=0;
				dout(10,0);//报警故障指示灯关，报警信号关
				dout(2,0); //继电器输出报警
			}
			////////备妥检测输出///////
			if(!keymode)//非键盘模式  新定义的DO3为报警输出(影响停车的报警) DO8定义为备妥  非键盘模式时，备妥不输出给PLC，导致PLC启动信号不输出，从而仪表秤停止
			{
			///	if(!c1)dout(2,1);//关键盘、无AD报警则输出备妥。
			///	else dout(2,0);
				if((xram[46]==0||xram[46]==2)&&(!ready_flag)) dout(8,1);//无报警,则输出备妥信号 //P46=0 启动为开关量X3:3,4;设定值值由模拟量X6：1，2  修改过 p46=0和P46=2的情况都要考虑
				else dout(8,0);//报警时，不输出备妥
				if((xram[43]==1)&&(xram[46]==1)) status[0]=0x31;//P43=1 有串行通讯 =0无串行通讯 p46=1 启动及设定值来源由串口输出 串口向上输出备妥信息//待改有问题
				else status[0]=0x30;
				
			}
			else//键盘模式  //键盘模式无需输出备妥信号
			{
			///	dout(2,0); 
				dout(8,0);//开键盘关备妥
				status[0]=0x30; //设定值由键盘输入，启动和停止也来自键盘
			}
			if((dis_dissigal_flag==1)&&(menu_exit_flag==1)) //许多函数用到了 //这些标志位控制某些函数的执行时间
			{
				dis_sec_count++;
				if(dis_sec_count>=2) //200ms
				{
					dis_dissigal_flag=0; //在主显示函数用
					menu_exit_flag=0;
					dis_sec_count=0;
				}
			}
			if((xram[43]==1)&&(M485REI ==1)&&(M485DEI==1))//P43位1 有串口通讯 //RE =1(PJ4),DE =1(PJ3) 此时发送模式     //使能RE=0 DE =1  485发送3s,接下来是接受 。485接收和发送频率为3s
			{
				RE_DE_Count++;
				if(RE_DE_Count==30)
				{
					USART_ITConfig(USART3, USART_IT_TC, DISABLE);//disable TX 关闭发送        //关闭发送完成中断
					USART3->CR1|=1<<2;    //接收缓冲区非空中断使能;		//enable RX  打开接受（这里打开，在哪里关闭了接收）   //关闭的是发送  
					USART_ClearFlag(USART3,USART_SR_RXNE);
					M485REI =0;			//RE enable ;DE disenable // RE =0  DE=0 /可以接收不能发送
                    M485DEI =0;					
				}
			}
			else RE_DE_Count=0;			
			/*************************************保护停止控制****************************************/
// 			if((!din(2))&&(run)&&(keymode)&&(xram[46]==0)) stop();//DI3释放开关打开//运行//键盘模式//P46=0:启动和设定值均来自端口 （有问题）待改 是否需要判断P46 
			if((!din(2))&&(run)&&(keymode)) stop();//改过，与上一语句对照
			if((!din(2))&&(din(1))&&(run)&&(!keymode)&&((xram[46]==0)||(xram[46]==2))) stop(); //run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0;  DI3  DI2
			if((run)&&((c1)||(c2)))
			 stop();//运行////称重传感器输入出错////速度传感器输入出错   串口时还停吗？ 问题
			/*************************************非键盘模式启停控制（启停端口控制）****************************************/
			if((!run)&&din(1)&&din(2)&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//关闭//DI2 启动和停止//DI3闭合//停止键盘模式//启动由端口，设定值由键盘或是端口   DI3  DI2
			{
				start();//启动开关量输入和4-20mA输出   // 1.非键盘模式  启动来自端口   
			}
			if((run)&&(!din(1))&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//运行//DI2启动和停止//停止键盘模式//启动由端口，设定值由键盘或是端口  按了秤停止键
			{
				stop();//启动开关量输入和4-20mA输出   
			}
			/***************************************键盘模式停止启动*******************************************/
			  if(keymode==1)//键盘模式
			 {   mhang0=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_0); //读取行
				 mhang1=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_1); 
				 mhang2=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_2); 
				 hang_1=1;//
				 hang_2=1;//
				if(run)//如果在运行状态
				{
					hang_0=0;
					if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)
					{
						delay_us(10);
						if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)stop();
					}
				}
				else//在停止模式时
				{
					hang_0=0;
					if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfd)
					{
						delay_us(10);
						if(((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfd)&&(!run))start();
					}
				}
				 
				  hang_0=mhang0;hang_1=mhang1;hang_2=mhang2;
			 }
        /**************************** CPU外给定码值的处理*********************************/
			anainput=flow_set();//外给定  返回码值   处理方式待改
      if((!keymode)&&(xram[46]==0))//系统进入停止键盘方式，0 设定值得来源由模拟量X6：1，2输入    启动为开关量X3：3，4   //CPU外给定设定值流量的处理
			{
				lsvalue=(anainput-settare)*setpra*input_rate;//input_rate 手动输入//电流对应的吨数 setpra单位码值对应的流量（t/h） input_rate(手动输入比例值)
				if(lsvalue>0)//流量值滤波
				{
					setvalue=setvalue_1+ap*(lsvalue-setvalue_1);//ap=0.5/P54  P54流量输入滤波时间
					setvalue_1=setvalue;
				}
				else setvalue=0;
			}
			/*************************************串口流量给定启停处理****************************************/
			if((xram[43]==1)&&(xram[46]==1)&&(!keymode)&&(r_flag=='Y'))//P43有串口通信//启动和设定值均来自串口输入//停止键盘  //串口用
			{
				if((lsset=ser_setvalue())!=-1)//串口给定
					setvalue=lsset;//串口流量给定
				if((rs_temp[3]=='K')||(rs_temp[3]=='k'))
				{
					if((rs_temp[2]==0x31)&&(run==0)) //2 启动和停止  //2.非键盘模式 启停来自串口
						start();
					else if((rs_temp[2]==0x30)&&(run==1))
						stop();
				}
			}
	
    /*******************************流量的计算处理*******************************************/  //流量均值的求取

			if(run)//运行状态下，计算流量值
			{   
				ctrealvalue=press*speedvalue*3.6*xram[36];//t/h   流量累计用
				realvalue=pressvalue*speedvalue*3.6*xram[36];//pressvalue载荷 //p36实物标定调整系数：p36=p36*实际累计值/显示累计值  流量值（为求下边流量值） kg/度* 度/s *3.6 =t/h(反馈流量值)   显示用
				lsreal=Xn*speedvalue*3.6*xram[36];  //p36实物标定调整系数：p36=p36*实际累计值/显示累计值  （挡板控制） kg/度* 度/s *3.6 =t/h （进入流量值控制挡板）
			}
			else {ctrealvalue=0,realvalue=0,lsreal=0;}//停止时流量清零
			
			////////////////////显示用////////////////////
			real=real_1+ai*(realvalue-real_1);//流量值滤波
			real_1=real;
			for(i=1;i<20;i++)avernum[i-1]=avernum[i]; //数组左移
			avernum[20-1]=real;//ak=30
			tracevalue=0;
			for(i=0;i<20;i++)tracevalue+=avernum[i];//求取平均值
			tracevalue/=20;//显示用 流量反馈
			delay_us(1);

     /*******************************挡板控制调节*******************************************/ 
     if((run)&&(!calflg))//运行并且非标定模式  
    {
			////////////////////驱动和应答////////////////////
	  runflag_count++;
	  if(runflag_count>=5)runflag_count=5;
	  dout(7,1);//秤启动   秤的驱动信号 转子秤点机启动
	  if(!xram[9]){speed_dis=xram[5];}//速度传感器无效，速度由P5决定上边已给speedvalue用于计算，speed_dis用于显示 //无用
	  if(xram[9]){if(speedcode>5)dout(0,1);}//1速度传感器有效，speedcode 100ms速度计数的码值   DO1=1 运行状态  500ms有了速度之后，(应答信号输出）   待改
	  else {if(runflag_count>=5)dout(0,1);} //应答信号输出   //待改
	  if(modecount==1)//刚启动控制模式1 关挡板 开转子速度 （剩余的料漏下去）
	  {
       ctrloutb=daminb;//关挡板
    }
	  else if(modecount==2)  //开挡板和转子速度  分隔单元开始记分隔单元的重量
	  {   
      ctrloutb=xram[73]*boardcal;//P73容积方式驱动电流 4096/20.88  最大码值对应的最大电流//待改   （以容积方式控制挡板开度）         				
    }
    else if(modecount==3)  //容积方式（手动模式）挡板的控制  速度的控制放在计数器里
			{   
				if(avmode||(!avmode))//容积方式。在重量工作当中，启动开始时，先按一下控制一段时间
				{  
					ctrloutb=xram[73]*boardcal;//P73容积方式驱动电流 4096/22.9  最大码值对应的最大电流//待改   
					ctrloutb+=avdanumb;
					if(ctrloutb<=daminb) ctrloutb=daminb;
					if(ctrloutb>=damaxb) ctrloutb=damaxb;
					if(ctrloutb>4095)    ctrloutb=4095;   //容积方式挡板开度控制
				}
				else if((xram[38]==1)&&(h5==0)){}//偏差范围内不调速P38偏差控制选择：当选择0时，在任何时间内调节点击速度。当选择1时，在偏差范围内不调节电机速度  在偏差不超限时
				else//重量工作方式                               //选择此参数可减小电机震荡，使得物料运行平稳。 
				{
	////			ek1=ek;
					if((bflag==0)||(((zb-zi)>0)&&bflag))//批量生产标志位0 bsetend 5*set_value/3600//流量设定值  ((zb-zi)>bsetend)&&bflag)  zb批量生产设定值 zi批量生产增量
					{     
						ek=setvalue-lsreal;//设定值-流入转子秤反馈值=偏差 
	////			danum=kpi*(ek*ki-ek1);//ek1上一时刻的偏差 B/t/h*(t/h-t/h) =B  积分抑制超调 ki =1+1/P12  皮带秤
						danumb=(kp+ki+kd)*ek+(-kp-2*kd)*ek1+kd*ek2;//增量控制
						ctrloutb+=danumb;
						if(ctrloutb<=daminb)ctrloutb=daminb;//码值最小值
						if(ctrloutb>=damaxb)ctrloutb=damaxb;//码值最大值
						if(ctrloutb>4095)ctrloutb=4095;
						ek2=ek1;
						ek1=ek;
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //批量生产结尾控制  去掉
					{
						if(bctr==0)
						{
							danumb=ctrloutb/220;
							bctr=1;
						}
						ctrloutb-=danumb;//100ms的控制周期  缓慢的减小
						if(ctrloutb<(daminb+300)) ctrloutb=daminb+300;
						else if(ctrloutb>damaxb) ctrloutb=damaxb;
						if(ctrloutb>4095)ctrloutb=4095;
					}
			 }
			 
		 }
  }
		else if(!calflg)//if((run==0)&&(!calflg)) //停止运行
		{
			runflag_count=0;
			dout(0,0);//运行状态关（应答）
			dout(7,0);//转子秤关闭 //启动  驱动
			ctrlout=ctrlana1;////停止输出最小值mA
			ctrloutb=ctrlana2;//停止挡板输出最小值mA
			if(!xram[9]){speedvalue=0;speed_dis=0;}//速度开关无效时
			plusesum=0;//累计脉冲数清零
		    modecount=0;//模式0 没启动
			plusecount=0;//计数中断的计量
		}
	/***************************************100ms流量求取*******************************************/
		if(calflg)//标定标记  //容积方式运行
		{
			total_temp=0;                                                       //P42 0:当荷重下限L2报警时，累计计数器继续累计//1:当荷重下限L2报警时，累计量不计数
			if((pressvalue<xram[24])&&(xram[42]==1)){delay_us(1);} //荷重小于荷重下限与当荷重下限L2报警时，累计计数器不累计
			else total_cal+=ctrealvalue*0.000027777778*xram[25];//求累计量//每100ms累计一次数值//p25 =p25(原值)*称量实际值/显示累计值    注：（显示累计值）测量累计值  0.000027 =1/36000
		}
		else if((l2==1)&&(xram[42]==1))//荷重小于荷重下限与当荷重下限L2报警时，累计量清零  //1累计量不累积
			{total_temp=0;}  
		else
			{total_temp=ctrealvalue*0.000027777778*xram[25];} //p25 =p25(原值)*称量实际值/显示累计值注：（显示累计值）测量累计值（正常）//1h=3600s=36000（100ms）

/***************************************累计量*******************************************/
	    z1_temp+=total_temp;//total_temp 100ms的累计值//total_temp流量值 t  //累计的流量值
// 			z1 =0;
// 			z1_temp =0;
// 			totalshift[0]=0;
// 			totalshift[1]=0;
// 			totalshift[2]=0;
			
		if(fabs(z1_temp)>=xram[26])//计数器脉冲单位
		{
			if(z1_temp>0)
			{
				z1+=xram[26];//计数器脉冲单位单位重量
				totalshift[shift]+=xram[26];//计数器脉冲单位 班累计  班组累计
				z1_temp-=xram[26];
			}
			else//什么情况下出现
			{
				z1-=xram[26];
				totalshift[shift]-=xram[26];//计数器脉冲单位 班累计  
				z1_temp+=xram[26];		
			}
			if(npulse)dout(6,1);//累计量//npulse=(uchar)(pramc[27]/100+0.9);//计算累计量脉冲输出时间
			pulsecount=npulse;	//时间到 dout(6,0)	
		}
		if((bflag==1)&&run)//批量成产标志位  //批量成产检测停止
		{
			zi+=total_temp;//zi =0  增量
			zd-=total_temp;//zd =0  减量
			if(zi>=zb) //增量大于设定值 则输出停止
			stop();	
		}

		if(clearflag1)//总吨位清零
		{
			totalshift[shift]+=z1_temp;
			z1=0;//总累计量
			z1_temp=0;//累计缓冲区
			clearflag1=0;

		}
		if(z1>=9999999){z1-=9999999;}//

    //////////////////////////////////////////////////////////////////////
	if(!calflg){analogout();}//没有在标定程序时执行  当前模拟反馈输出  //执行标定时，没有执行
	
	if(mainboardflagb==0)//挡板控制调节
    {
	   DAC7612_2(0,ctrloutb);
    }//挡板控制
	 
  if(mainboardflag==0&&deputyboardflag==0)//当主板流量输出校准时，此标志位置1 不可以输出
   {
	    DAC7612(0,ctrlout);
   }//速度控制输出   
	change_shift();//换班 确定那个班 
// 	cli();//关中断                                  有问题  //待改
// 	TIMSK1=(1<<OCIE1A);//bit1 输出比较中断使能位。   有问题
	
	 /////////////SysTick延时寄存器数据还原////////
	  SysTick->LOAD=t_val;
	  SysTick->VAL=0x00;
	  SysTick->CTRL=t_ctrl;

   } 	 	
	   
} 	 	


/*********************************外部中断（AD7730读）********************************/
void EXTI15_10_IRQHandler(void)
{
// 	delay_ms(10);//消抖
   if( EXTI_GetITStatus( EXTI_Line15)!=RESET)
   { 
		uint8_t i=0;
		Timedata=0;
		c4=0;
		/////////////SysTick延时寄存器数据备份////////   
		e_val=SysTick->VAL;//不为0 则 16位 为0 为0 16位清零
		if(e_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器1
		e_ctrl=SysTick->CTRL;  
			 
		if(ad_flag==0)//ad标志位
		{
				if(INT7_flag==0)//中断标志位
				{
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//数组向左移位//最低位数组被覆盖
					buf7730=AD7730_READ();//读重力值
					ad7730_filter[25]=buf7730;//给到滤波器
				}
				else
				{
					INT7_flag=0;
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//数组向左移位
					ad7730_filter[25]=buf7730;//上一次出来的值
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//数组向左移位
					buf7730=AD7730_READ();
					ad7730_filter[25]=buf7730;
				}
		}
		else//在计算刷新程序时，一直计算下次代码，只能保留一次不丢失
		{
					INT7_flag=1;
					buf7730=AD7730_READ();
		}	
	/////////////SysTick延时寄存器数据还原////////
		 SysTick->LOAD=e_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e_ctrl;
	
		 EXTI_ClearITPendingBit(EXTI_Line15);  //清除LINE15上的中断标志位
  }	   
}

/*********************************串口1中断服务程序（接收、发送）********************************/
//void USART1_IRQHandler(void)
//{
//	u8 TEXT_TO_SEND[]={"tongxunxieyichengong"};
//	
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//检查指定的USART中断发生与否	
//	{
//		  uint8_t i=0,j=0;
//		  for(i=14;i>0;i--)//16个数组 
//	  	{USART_RX_BUF[i]=USART_RX_BUF[i-1];}//数组向右移动 
//		  USART_RX_BUF[0]=USART_ReceiveData(USART1); //接收
//    	if(((USART_RX_BUF[14]=='F')||(USART_RX_BUF[14]=='f'))&&((USART_RX_BUF[10]=='T')||(USART_RX_BUF[10]=='t'))&&((USART_RX_BUF[2]=='N')||(USART_RX_BUF[2]=='n')))//每一串数据接收完成  
//			{
//			  USART1->CR1|=0x0008;//使能发送
//				j=sizeof(TEXT_TO_SEND);
//        for(i=0;i<=j;i++)
//	      {
//		      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//		      USART_SendData(USART1,TEXT_TO_SEND[i] );
//	      }
//	      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//		  }
//  }	
//}
/*********************************串口3中断服务程序（接收、发送）********************************/
void USART3_IRQHandler(void)                	//串口3中断服务程序
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
  {
	  uint8_t i=0;
	  float series_ls=0;
		for(i=11;i>0;i--)//12个数组 0:前面全部数据二进制累加和 1：仪表地址(16进制) （0,1都是二进制码） 2：0 1 启动和停止命令
		{rs_temp[i]=rs_temp[i-1];}//数组向右移动  //3:"K" 4-10:7位流量设定值   11:"S"  
		rs_temp[0]=USART_ReceiveData(USART3);//(USART3->DR);//接收
		if(((rs_temp[11]=='S')||(rs_temp[11]=='s'))&&((rs_temp[3]=='K')||(rs_temp[3]=='k')))//每一串数据接收完成  
		{
			if(rs_temp[1]!=xram[44])return;//P44串行通讯地址：在一组总线中，每台仪表有唯一通讯地址  通讯地址错误 退出  //不启动发送
			r_crc=0;
			for(i=1;i<=11;i++)
			{
				r_crc+=rs_temp[i];//1-11相加
			}
			if(r_crc==rs_temp[0])//和校验成功
			{
			  r_flag='Y';t_crc=0;
				addr=rs_temp[1];//仪表地址
				//////////////////=============/////////////////////////
				//////////////////待发送数据处理////////////////////////
				//////////////////=============/////////////////////////
				ts_temp[0]='S';
				series_ls=tracevalue;//滤波均值处理后的流量值
				put_series_data(series_ls,xram[0]);//数据  小数位数  //7位数据包括小数点
				for(i=0;i<7;i++)ts_temp[1+i]=tra_temp[i];//流量值7位（包括小数点） 占7个数组 装入1—7数组   ts_temp[1]为最高位
				ts_temp[8]='=';ts_temp[9]='P';
				series_ls=z1+z1_temp;//总累计量 
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);//P34//累计量小数位数
				for(i=0;i<7;i++)ts_temp[10+i]=tra_temp[i];//10-16
				ts_temp[17]='=';ts_temp[18]='B';
// 				if(fser==0)                              //改
				series_ls=totalshift[shift]+z1_temp;//班累计
// 				else//fser=1 换班时刻
// 				{
// 					if(shift==0)
// 				  series_ls=totalshift[shift+2];
// 					else
// 					series_ls=totalshift[shift-1];//
// 					lsfser=1;//变量没有用
// 				}
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);
				for(i=0;i<7;i++)ts_temp[19+i]=tra_temp[i];//19-25  班累计
				
				ts_temp[26]='=';ts_temp[27]='V';//
				series_ls=speedvalue;//速度值
				put_series_data(series_ls,xram[4]);	
				for(i=0;i<7;i++)ts_temp[28+i]=tra_temp[i];//28-34
				
				ts_temp[35]='=';ts_temp[36]='W';//荷重
				series_ls=pressvalue;
				put_series_data(series_ls,xram[2]);	
				for(i=0;i<7;i++)ts_temp[37+i]=tra_temp[i];//37-43
				
				ts_temp[44]='=';ts_temp[45]='K';
				if(run)//运行信号  status[0]是备妥：  status[5]、status[6]是班累计  //事件信息串口发送  发送ACSII
					status[1]=0x31;//启停状态
				else
					status[1]=0x30;
				if(h9)		//跑偏或现场控制
					status[2]=0x31;
				else
					status[2]=0x30;
				if((h2)||(l2))//皮带实际负载超出设定值极限
					status[3]=0x31;
				else
					status[3]=0x30;
				if(h5)//偏差超出
					status[4]=0x31;
				else
					status[4]=0x30;
				for(i=0;i<7;i++)tra_temp[i]=status[i];
				for(i=0;i<7;i++)ts_temp[46+i]=tra_temp[i];//46-52
				
				ts_temp[53]='=';ts_temp[54]='C';
				ts_temp[55]=r_flag;//上一次接受正确发送Y，否则发送N
				ts_temp[56]=xram[44];//P44代表本机地址
// 				ts_temp[56]='1';
				for(i=0;i<4;i++)ts_temp[i+57]=' ';//发送4个空格 57-60
				for(i=0;i<61;i++)t_crc+=ts_temp[i];
				t_crc=t_crc+0x24;//0x24 =$  
				ts_temp[61]=t_crc;
// 				ts_temp[61]='A';
				//////////////////=============/////////////////////////
				//////////////////=============/////////////////////////
// 				UCSR1B|=(1<<TXCIE1);//发送结束中断使能
// 				UCSR1A&=0xbf;//发送结束标志位清零
// 				PORTJ|=0x18;//DE =1 RE =1 使能485发送，关闭485接受
// 				UDR1=0x24;//发送前  //启动发送  $
				M485DEI=1;M485REI=1;//DE =1 RE =1 使能485发送，关闭485接受
				USART3->CR1|=0x0008;//使能发送
				USART_ClearFlag(USART3, USART_FLAG_TC);
				USART_ITConfig(USART3, USART_IT_TC, ENABLE);return; //发送中断使能 
			}
			else
			{
				r_crc=0;//接受校验变量
				r_flag='N';//接受校验失败	
			}
		}
	}
			
 
// 	 USART_ITConfig(USART1, USART_IT_TC, DISABLE);  
 //  USART_SendData(USART1,usar1_temp[ab] );	
//   USART_ClearFlag(USART1, USART_FLAG_TC);

// 		UCSR1A&=0xbf;////发送结束标志位清零	
 if( USART_GetITStatus(USART3, USART_IT_TC) == SET  ) //发送完成中断   发送给了30s的时间，时间到，系统处于接收状态
{ 
    USART_SendData(USART3,0X24);       
	if((r_flag!=0x11)&&(addr==xram[44]))//检验成功后才发送，r_flag ="Y"
	{
		switch(sart)
		{
			case 0:USART_SendData(USART3, ts_temp[0]);sart++;break;
			case 1:USART_SendData(USART3, ts_temp[1]);sart++;break;
			case 2:USART_SendData(USART3, ts_temp[2]);sart++;break;
			case 3:USART_SendData(USART3, ts_temp[3]);sart++;break;
			case 4:USART_SendData(USART3, ts_temp[4]);sart++;break;
			case 5:USART_SendData(USART3, ts_temp[5]);sart++;break;
			case 6:USART_SendData(USART3, ts_temp[6]);sart++;break;
			case 7:USART_SendData(USART3, ts_temp[7]);sart++;break;
			case 8:USART_SendData(USART3, ts_temp[8]);sart++;break;
			case 9:USART_SendData(USART3, ts_temp[9]);sart++;break;
			case 10:USART_SendData(USART3, ts_temp[10]);sart++;break;
			case 11:USART_SendData(USART3, ts_temp[11]);sart++;break;
			case 12:USART_SendData(USART3, ts_temp[12]);sart++;break;
			case 13:USART_SendData(USART3, ts_temp[13]);sart++;break;
			case 14:USART_SendData(USART3, ts_temp[14]);sart++;break;
			case 15:USART_SendData(USART3, ts_temp[15]);sart++;break;
			case 16:USART_SendData(USART3, ts_temp[16]);sart++;break;
			case 17:USART_SendData(USART3, ts_temp[17]);sart++;break;
			case 18:USART_SendData(USART3, ts_temp[18]);sart++;break;
			case 19:USART_SendData(USART3, ts_temp[19]);sart++;break;
			case 20:USART_SendData(USART3, ts_temp[20]);sart++;break;
			case 21:USART_SendData(USART3, ts_temp[21]);sart++;break;
			case 22:USART_SendData(USART3, ts_temp[22]);sart++;break;
			case 23:USART_SendData(USART3, ts_temp[23]);sart++;break;
			case 24:USART_SendData(USART3, ts_temp[24]);sart++;break;
			case 25:USART_SendData(USART3, ts_temp[25]);sart++;break;
			case 26:USART_SendData(USART3, ts_temp[26]);sart++;break;
			case 27:USART_SendData(USART3, ts_temp[27]);sart++;break;
			case 28:USART_SendData(USART3, ts_temp[28]);sart++;break;
			case 29:USART_SendData(USART3, ts_temp[39]);sart++;break;
			case 30:USART_SendData(USART3, ts_temp[30]);sart++;break;
			case 31:USART_SendData(USART3, ts_temp[31]);sart++;break;
			case 32:USART_SendData(USART3, ts_temp[32]);sart++;break;
			case 33:USART_SendData(USART3, ts_temp[33]);sart++;break;
			case 34:USART_SendData(USART3, ts_temp[34]);sart++;break;
			case 35:USART_SendData(USART3, ts_temp[35]);sart++;break;
			case 36:USART_SendData(USART3, ts_temp[36]);sart++;break;
			case 37:USART_SendData(USART3, ts_temp[37]);sart++;break;
			case 38:USART_SendData(USART3, ts_temp[38]);sart++;break;
			case 39:USART_SendData(USART3, ts_temp[39]);sart++;break;
			case 40:USART_SendData(USART3, ts_temp[40]);sart++;break;
			case 41:USART_SendData(USART3, ts_temp[41]);sart++;break;
			case 42:USART_SendData(USART3, ts_temp[42]);sart++;break;
			case 43:USART_SendData(USART3, ts_temp[43]);sart++;break;
			case 44:USART_SendData(USART3, ts_temp[44]);sart++;break;
			case 45:USART_SendData(USART3, ts_temp[45]);sart++;break;
			case 46:USART_SendData(USART3, ts_temp[46]);sart++;break;
			case 47:USART_SendData(USART3, ts_temp[47]);sart++;break;
			case 48:USART_SendData(USART3, ts_temp[48]);sart++;break;
			case 49:USART_SendData(USART3, ts_temp[49]);sart++;break;
			case 50:USART_SendData(USART3, ts_temp[50]);sart++;break;
			case 51:USART_SendData(USART3, ts_temp[51]);sart++;break;
			case 52:USART_SendData(USART3, ts_temp[52]);sart++;break;
			case 53:USART_SendData(USART3, ts_temp[53]);sart++;break;
			case 54:USART_SendData(USART3, ts_temp[54]);sart++;break;
			case 55:USART_SendData(USART3, ts_temp[55]);sart++;break;
			case 56:USART_SendData(USART3, ts_temp[56]);sart++;break;
			case 57:USART_SendData(USART3, ts_temp[57]);sart++;break;
			case 58:USART_SendData(USART3, ts_temp[58]);sart++;break;
			case 59:USART_SendData(USART3, ts_temp[59]);sart++;break;
			case 60:USART_SendData(USART3, ts_temp[60]);sart++;break;
			case 61:USART_SendData(USART3, ts_temp[61]);sart++;break;//校验和
      case 62:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//串口发送使能关闭  DE=0 RE=0  485 关闭发送 使能接收
 			default:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//串口发送使能关闭  DE=0 RE=0  485 关闭发送 使能接收
		}
	}
	
  }

} 

/*********************************串口2中断服务程序（发送完成中断）********************************/
 void USART2_IRQHandler(void)   
{
  if( USART_GetITStatus(USART2, USART_IT_TC) == SET  ) //发送完成中断
  {
		if(xram[57]==1) //打印选择
		{
			switch(tx0_count)//发送计数器  //发送 时间  1班累计 2班累计 3班累计 总累计
			{
				case 0:USART_SendData(USART2, prn_time[0]);tx0_count++;break;//打印时间  年     
				case 1:USART_SendData(USART2, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART2, '-');tx0_count++;break;
				case 3:USART_SendData(USART2, prn_time[2]);tx0_count++;break;          //月
				case 4:USART_SendData(USART2, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART2, '-');tx0_count++;break;	
				case 6:USART_SendData(USART2, prn_time[4]);tx0_count++;break;          //日
				case 7:USART_SendData(USART2, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART2, 0x0d);tx0_count++;break; //回车
				case 9:USART_SendData(USART2, 0x0a);tx0_count++;break; //换行
				case 10:USART_SendData(USART2, prn_time[6]);tx0_count++;break;         //时
				case 11:USART_SendData(USART2, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART2, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART2, prn_time[8]);tx0_count++;break;         //分
				case 14:USART_SendData(USART2, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART2, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART2, prn_time[10]);tx0_count++;break;        //秒
				case 17:USART_SendData(USART2, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART2, 0x0d);tx0_count++;break;//回车 Enter
				case 19:USART_SendData(USART2, 0X0a);tx0_count++;break;//换行
				case 20:USART_SendData(USART2, 0X3d);tx0_count++;break;//=
				case 21:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 22:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 23:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 24:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 25:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 26:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 27:USART_SendData(USART2, 0X3d);tx0_count++;break;
				case 28:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 29:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 30:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 31:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 32:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 33:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 34:USART_SendData(USART2, 0X3d);;tx0_count++;break;
				case 35:USART_SendData(USART2, 0X3d);;tx0_count++;break;//=
				case 36:USART_SendData(USART2, 'T');;tx0_count++;break; //TOTAL:113213132t
				case 37:USART_SendData(USART2, 'O');;tx0_count++;break; //SHIFT:1
				case 38:USART_SendData(USART2, 'T');;tx0_count++;break;
				case 39:USART_SendData(USART2, 'A');;tx0_count++;break;
				case 40:USART_SendData(USART2, 'L');;tx0_count++;break;
				case 41:USART_SendData(USART2, ':');;tx0_count++;break;
				case 42:USART_SendData(USART2, prnbuf1[7]);tx0_count++;break;//prnbuf1  1班组累计量
				case 43:USART_SendData(USART2, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART2, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART2, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART2, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART2, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART2, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART2, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART2, 0x0d);tx0_count++;break;//回车换行
				case 52:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART2, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART2, 'H');tx0_count++;break;
				case 55:USART_SendData(USART2, 'I');tx0_count++;break;
				case 56:USART_SendData(USART2, 'F');tx0_count++;break;
				case 57:USART_SendData(USART2, 'T');tx0_count++;break;
				case 58:USART_SendData(USART2, ':');tx0_count++;break;
				case 59:USART_SendData(USART2, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART2, 0x0d);tx0_count++;break;//回车
				case 61:USART_SendData(USART2, 0x0a);tx0_count++;break;//换行
				case 62:USART_SendData(USART2, 0x2d);tx0_count++;break;//-
				case 63:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 64:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 65:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 66:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 67:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 68:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 69:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 70:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 71:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 72:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 73:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 74:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 75:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 76:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 77:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 78:USART_SendData(USART2, 'T');tx0_count++;break;//TOTAL
				case 79:USART_SendData(USART2, 'O');tx0_count++;break;
				case 80:USART_SendData(USART2, 'T');tx0_count++;break;
				case 81:USART_SendData(USART2, 'A');tx0_count++;break;
				case 82:USART_SendData(USART2, 'L');tx0_count++;break;
				case 83:USART_SendData(USART2, ':');tx0_count++;break;
				case 84:USART_SendData(USART2, prnbuf2[7]);tx0_count++;break;//prnbuf2  班组2
				case 85:USART_SendData(USART2, prnbuf2[6]);tx0_count++;break;
				case 86:USART_SendData(USART2, prnbuf2[5]);tx0_count++;break;
				case 87:USART_SendData(USART2, prnbuf2[4]);tx0_count++;break;
				case 88:USART_SendData(USART2, prnbuf2[3]);tx0_count++;break;
				case 89:USART_SendData(USART2, prnbuf2[2]);tx0_count++;break;
				case 90:USART_SendData(USART2, prnbuf2[1]);tx0_count++;break;
				case 91:USART_SendData(USART2, prnbuf2[0]);tx0_count++;break;
				case 92:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 93:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 94:USART_SendData(USART2, 0x0a);tx0_count++;break;			
				case 95:USART_SendData(USART2, 'S');tx0_count++;break;
				case 96:USART_SendData(USART2, 'H');tx0_count++;break;
				case 97:USART_SendData(USART2, 'I');tx0_count++;break;
				case 98:USART_SendData(USART2, 'F');tx0_count++;break;
				case 99:USART_SendData(USART2, 'T');tx0_count++;break;
				case 100:USART_SendData(USART2,':');tx0_count++;break;
				case 101:USART_SendData(USART2, 0x32);tx0_count++;break;
				case 102:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 103:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 104:USART_SendData(USART2, 0x2d);tx0_count++;break;//-
				case 105:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 106:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 107:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 108:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 109:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 110:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 111:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 112:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 113:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 114:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 115:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 116:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 117:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 118:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 119:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 120:USART_SendData(USART2, 'T');tx0_count++;break;
				case 121:USART_SendData(USART2, 'O');tx0_count++;break;
				case 122:USART_SendData(USART2, 'T');tx0_count++;break;
				case 123:USART_SendData(USART2, 'A');tx0_count++;break;
				case 124:USART_SendData(USART2, 'L');tx0_count++;break;
				case 125:USART_SendData(USART2, ':');tx0_count++;break;
				case 126:USART_SendData(USART2, prnbuf3[7]);tx0_count++;break;//prnbuf3  班组3
				case 127:USART_SendData(USART2, prnbuf3[6]);tx0_count++;break;
				case 128:USART_SendData(USART2, prnbuf3[5]);tx0_count++;break;
				case 129:USART_SendData(USART2, prnbuf3[4]);tx0_count++;break;
				case 130:USART_SendData(USART2, prnbuf3[3]);tx0_count++;break;
				case 131:USART_SendData(USART2, prnbuf3[2]);tx0_count++;break;
				case 132:USART_SendData(USART2, prnbuf3[1]);tx0_count++;break;
				case 133:USART_SendData(USART2, prnbuf3[0]);tx0_count++;break;
				case 134:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 135:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 136:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 137:USART_SendData(USART2, 'S');tx0_count++;break;
				case 138:USART_SendData(USART2, 'H');tx0_count++;break;
				case 139:USART_SendData(USART2, 'I');tx0_count++;break;
				case 140:USART_SendData(USART2, 'F');tx0_count++;break;
				case 141:USART_SendData(USART2, 'T');tx0_count++;break;
				case 142:USART_SendData(USART2, ':');tx0_count++;break;//3
				case 143:USART_SendData(USART2, '3');tx0_count++;break;//3
			  case 144:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 145:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 146:USART_SendData(USART2, 0x2d);tx0_count++;break;//-
				case 147:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 148:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 149:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 150:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 151:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 152:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 153:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 154:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 155:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 156:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 157:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 158:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 159:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 160:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 161:USART_SendData(USART2, 0x2d);tx0_count++;break;
				case 162:USART_SendData(USART2, 'S');tx0_count++;break;
				case 163:USART_SendData(USART2, 'U');tx0_count++;break;
				case 164:USART_SendData(USART2, 'M');tx0_count++;break;
				case 165:USART_SendData(USART2, ':');tx0_count++;break;
				case 166:USART_SendData(USART2, ' ');tx0_count++;break;
				case 167:USART_SendData(USART2, ' ');tx0_count++;break;
				case 168:USART_SendData(USART2, prnbuf4[7]);tx0_count++;break;//prnbuf4 总的累计量 
				case 169:USART_SendData(USART2, prnbuf4[6]);tx0_count++;break;
				case 170:USART_SendData(USART2, prnbuf4[5]);tx0_count++;break;
				case 171:USART_SendData(USART2, prnbuf4[4]);tx0_count++;break;
				case 172:USART_SendData(USART2, prnbuf4[3]);tx0_count++;break;
				case 173:USART_SendData(USART2, prnbuf4[2]);tx0_count++;break;
				case 174:USART_SendData(USART2, prnbuf4[1]);tx0_count++;break;
				case 175:USART_SendData(USART2, prnbuf4[0]);tx0_count++;break;
				case 176:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 177:USART_SendData(USART2, 0x3d);tx0_count++;break;//=
				case 178:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 179:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 180:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 181:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 182:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 183:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 184:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 185:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 186:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 187:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 188:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 189:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 190:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 191:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 192:USART_SendData(USART2, 0x3d);tx0_count++;break;
				case 193:USART_SendData(USART2, 0x0d);tx0_count++;break;//回车 Enter	
				case 194:USART_SendData(USART2, 0x0a);tx0_count++;break;//换行
				case 195:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 196:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 197:USART_SendData(USART2, 0x0d);tx0_count++;break;
				case 198:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 199:tx0_count=0;USART_ITConfig(USART2, USART_IT_TC, DISABLE);break;
				default:tx0_count=0;USART_ITConfig(USART2, USART_IT_TC, DISABLE);break;
			}
		}
	
    }
	
}

void EXTI9_5_IRQHandler(void) //标定圈脉冲数
{
 if( EXTI_GetITStatus( EXTI_Line5)!=RESET)
 { 
		/////////////SysTick延时寄存器数据备份////////   
		e5_val=SysTick->VAL;//不为0 则 16位 为0 为0 16位清零
		if(e5_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//关闭计数器1
		e5_ctrl=SysTick->CTRL;  
	  EXTI_ClearITPendingBit(EXTI_Line5);  //清除LINE15上的中断标志位
//	EXTI_Set_Int(0);//关外部中断停止记圈数   
// 	  tare_s=0;
 if(calflg==1)
 {
   time_flag=1;
   EXTI_Set_Int(0);//关外部中断停止记圈数
  if(tare_cycle_flag==1)
  {
	  if(cycle_flag==1)//第一次进入中断开计数器1 计脉冲数
		{ 
			TIM_SetCounter(TIM1, 0);//计数器清零
			TIM_Cmd(TIM1, ENABLE);  //使能TIM1 //TIM1定时器使能	 标定时圈脉冲数才开	
			cycle_count=0;
			cycle_flag=0;	
		}			
		else
		{
			cycle_count++; 
//cycplussum+=TIM_GetCounter(TIM1);
//cycplussum+=tim1_count();
//TIM_SetCounter(TIM1, 0);
		}
		if(cycle_count>=xram[78])
	    {
        EXTI_Set_Int(0);//关外部中断停止记圈数
		    cycle_flag=1;
	      TIM_Cmd(TIM1, DISABLE);//关定时器
      }
  }
  else//tare_cycle_flag=0
  {
// 	  tareline=1;   //进入中断后判断是否从零点开始称重
	 if(tare_flag==1)
   { 
	     TIM_SetCounter(TIM1, 0);
       TIM_Cmd(TIM1, ENABLE);
       tare_count=0;
       tare_flag=0;
	 }			
	  else
    {
	     tare_count++;
			 TIM_SetCounter(TIM1, 0);
// 		 tarecount=0;
	  }
	  if(tare_count>=xram[78]) 
	  {
      EXTI_Set_Int(0);//关外部中断停止记圈数
	    tare_flag=1;
			TIM_Cmd(TIM1, DISABLE);//关定时器
// 	  tarecount=0;
    }
  }
// 	if(weight_check_flag==1)
// 	{
// 		 if(check_flag==1)
//       { 
//        check_count=0;
//        check_flag=0;
// 	    }			
// 	  else
//     {
// 	     check_count++;
// 	  }
// 	  if(check_count>=xram[78]) 
// 	  {
//       EXTI_Set_Int(0);//关外部中断停止记圈数
//       weight_check_flag=0;
//     }
//   }
  }
else
 { 
 	Zero_flag=1;
	tare_s=0;
    
 }
//   tareline=0; 
 
	/////////////SysTick延时寄存器数据还原////////
		 SysTick->LOAD=e5_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e5_ctrl;	
 }	   
}




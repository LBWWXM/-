#include "string.h"//strcpy������
#include  "stdio.h" //sprintf����
#include "definition.h"
#include "config.h"
#include "drivers.h"
#include "sub.h"
#include "delay.h"
#include "math.h"
#include "stdlib.h"


extern char disbuf[40];//��ʾ���浥Ԫ
extern u32 ad7730_filter[26];
extern u32 ad7730_code_result[26];
/*************************************************���ļ������Ķ���*************************************************************/

char chartemp[10][23];

volatile float setvalue=0,z1=0,z1_temp=0,z2=0,z2_temp=0,z3=0,z3_temp=0,z=0,z_temp=0,input_rate=1,sec=0,speed_v1=0,speed_code=0,
				servicepressvalue0=0,servicepressvalue1=0,tracevalue=0,pressvalue=0,speedvalue=0,avervalue=0,speedpra=0,zb=0,zd=0,zi=0,
				nload=1,tarevalue=0,realvalue=0,ctrealvalue,total_temp=0,total_cal=0,bsetend=0,ai=0,av=0,ap=0,ac=0,ao=0,setvaluetemp=0,lsreal=0, 
				ctrlana1=0,ctrlana2=0,presspra=1,danum=0,danumb=0,ek=0,ek1=0,ek2=0,ki=0,kpi=0,kp=0,kd=0,kpv=0,aq=0,ctrlout=0,ctrloutb=0,devval=0,vspeedpra=0,stoppressvalue=0,setpra=0,
				settare=0,z1clear=0,z2clear=0,z3clear=0,kjtotal=0,kj_temp=0,G=0,Xn,press=0,press_1=0,ms_secb,averpress[press_ak],real_1=0,Sepapulse=0,
				real=0,setvalue_1=0,analogpra=0,analog0=0,analog1=0,analogpra1=0,analog2=0,analogpra2=0,totalshift_temp[3]={0},avernum[20]={0},
                totalshift[3]={0},speed_dis=0,speed_v=0,speed_hz=0,lsvalue=0,lsset=0,lastday_total={0},sumquality=0,boardset=0,partmoment,angledive,
                speedcal,flowcal,boardcal,whtload,plusecount=0,speed_set,plusesum,tare_date[TARE_NUM];
volatile uint8_t warnflag=0,s2=0,s3=0,s4=0,s5=0,s6=0,s7=0,s9=0,c1=0,c2=0,c3=0,s5_flag=0,s6_flag=0,run=0,ms_sec=0,dis_dissigal_flag=0,tare_flag=0,//ԭs8��s9
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
volatile int16_t 	damin=0,damax=0,daminb=0,damaxb=0,avdanum=0,avdanumb;//ת���ٶ���ֵ  ������ֵ����
volatile uint32_t  	taretime=0,totalpulse=0,ontime=0,runtime=0,day_flag=0,addat=0,servicetime=0,buf7730=0,cycplussum1=0,cycss=0;
volatile u16 flow_data[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   //0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//CPU����������˲���Ԫ
volatile float xram[PRA_NUM],tare_date_result[TARE_NUM];//SRAM //0-128     
volatile float Q_quality[200],Q_lenth[200];//ת�ӳ� 
union SRAMDS3234  DS3234SRAM;//	DS3234 SPAM�����
struct time DS3234time[2];//DS3234 ʱ�����ֵ
 u8 mhang0, mhang1, mhang2;
//////////////////////////////////////////�жϱ�������/////////////////////////////////////////////
u32 t_val,t_ctrl,e_val,e_ctrl,s_val,s_ctrl,e5_val,e5_ctrl;   //��ʱ��//�ⲿ�ж�//�ٶȼ������ж�

					
int main(void)
{
   uint8_t i=0;
   CLI();//���ж�
// delay_ms (1000);
   config();//������Դ���������� IO���� ��ʱ�� ����������
   d_out=1;//���������״̬λ 1ʱ�������
   dout(0,0);dout(1,0);dout(2,0);dout(3,0);dout(4,0);dout(5,0);dout(6,0);dout(7,0);dout(8,0);//��������λ
   delay_ms (500);
   d_out=0;//�رտ��������
   clk_74 =0;
   delay_ms (5);
   clk_74 =1;//��������Ч
   delay_ms(100);
   Init_LCD16032_DS3234();//��ʾ����ʼ��
   ad7730_init();//AD7730��ʼ��
   DS3234_READSRAM_VALUE(); //�����������SRAM�ĸ��ֲ���readpra(); //���Ĭ�ϱ�������  DS3234
   check_pra();//�洢�����Ƿ���ȷ  ����ȷ��Ĭ�ϲ���д��EEPROM //���� �������Ĭ�ϲ�����������DS3234��������ʼ�� 
   CLI();
   readpra();   //�ڶ���eeprom
   readpra1();	
   CLI();	
   display_check();//��ʾ��� �������key_input����������Ӧ�����ݻ�װ��Ĭ�ϲ���  ����������ʼʱ��
   CLI();
   calc_dat();	//ϵ������
   if(z1<0){z1=0;z1_temp=0;}//z1���ۼ��� z1_temp 100ms�ۼ���
   if(totalshift[0]<=0)totalshift[0]=0;//Z1//�����ۼ�
   if(totalshift[1]<=0)totalshift[1]=0;//Z2
   if(totalshift[2]<=0)totalshift[2]=0;//Z3�ۼ��� �洢��DS3234�У�12C887��
   run=running=0;//����λ 
   for(i=0;i<press_ak;i++)averpress[i]=0;//press_ak=20 //ѹ��ƽ�����浥Ԫ
   press=press_1=0;
   sumquality=addat*presspra-tarevalue;//���� kg  //û���� ���жϱ���
   dissignal();//�˵�������ʾ���ֽ��溯��
   d_out=1;//���
   if(!keymode)//�Ǽ���ģʽ  keymode =1 ����ģʽ  keymode =0 ͣ����ģʽ ��������ڶ�ʱ����Ҳ�У�
   {
	    dout(7,0);//��ֹͣ
		  dout(8,1);//�����ź����  �����Ѿ���ʼִ�У�֤��
      if((xram[43]==1)&&(xram[46]==1))//P43����ͨѶ��ַѡ��1�д���ͨѶ��0�޴���ͨѶ��//�����趨ֵ����Դ�����Դ��� 
		  status[0]=0x31;//�б���  //���ڷ�����
 		  else //����Ϊ������x3:3,4  �趨ֵ��ģ����x6:1,2��������ɼ��̸���
		  status[0]=0x30;//�ޱ���
   }
    else//����ģʽ  �������趨ֵ���м����趨
   {
	    dout(8,0); //����ģʽ ����Ҫ�������
		  status[0]=0x30; //���ڷ����� ���ޱ��ף�
   }
   delay_ms(1);
////d_out=1;//������������� //������ ����
   SEI();

 while(1)
{ 
  delay_us(3);
	check_day();//�������ʱ�� //����ʱ�䵽day_locking_falg=1 check_day_flag=1; xram[125]=1;����Ĭ�ϲ��� ������   ��дflash
	delay_us(3);
	recive_key();
	delay_us(3);
	if(((jgdis>1)||keyflag)){dissignal();jgdis=0;}//���̰���û�ɿ����� ��־λ����1  �������°�����ʾ
	else jgdis++;

	/*****************************������������***********************************///(ָʾ�Ʊ��� ���������)
//     if(din(0))clear_warn();//����������  ����Ӧ�� DI1 //��������溯����Ч   û�� û��
	if(maxwarn){dout(1,1);dout(11,1);} //���ֵ������������������ֵָʾ�Ʊ���  DO2
	else {dout(1,0);dout(11,0);}		   
	if(minwarn){dout(3,1);dout(12,1);}//��Сֵ�����������������Сֵָʾ�Ʊ���   DO3           
	else {dout(3,0);dout(12,0);}//�����������Сֵ��������أ���Сֵָʾ�Ʊ�����

	/********************************�¼���Ϣ���*************************************/
    ////////��ƫ���ش���/////// ��ƫ���Ǳ�ӵ�ֹͣ  Ƥ�����ã�ת�ӳӿ��ܲ���Ҫ
	if(din(2)) h9=0;//DI3=0 �ɽ���ƫ����//û����ƫ//�ͷ�    //Ƥ����ƫ���ֳ�����       ����Ϊ0   �����رպ�ʱ�� //DI3
	else//DI3=1//����ƫ  �����ضϿ�ʱ��                                         ��ƫ�ֳ�ȷ��  
	{
		h9=1;
		if(xram[41]==1)//1 ��ƫ���ضϿ�ʱ���Ǳ������������ֹͣ//0 ��ƫ���ضϿ�ʱ���Ǳ��������ֹͣ
		{
// 			r_temp[9]=0x30;//û���õ�������
			stop();//run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0; 
		}
	}
	////////ת�ӳӸ����¼� ////////  P10���ش��������ص�������  Ĭ��ֵ60����
	if((sumquality+tarevalue)>(xram[10]*1.1)){h4=1;l4=0;}//���ش���������*1.1/��Ч�����γ���//H4 ����ϵͳ����//�������ش��������ظ��ص�1.1��  66����
	else h4=0;
	
	
	if((sumquality+tarevalue)<(xram[10]*0.03))//���ش���������  1.8����
	{
		
			l4=1;//����ϵͳ���ع�С
			h4=0;
		
	}
	else l4=0;
	 ////////ת�ӳ������¼� //////// kg/h
	if((tracevalue>xram[21])&&(running>5))//�������� ���� ȷ���Ǳ�ȷʵ��������//P21�������� ����ʱ�����5S   ����ʱ�����
	{
		if(tracevalue>xram[21])//������������   
		{
			h1=1;l1=0;//ʵ��������������ֵ
			maxwarn=1;//�������������
			minwarn=0;
		}
	}
	else
	{
		maxwarn=0;
		h1=0;
	}
	if((tracevalue<=xram[22])&&(running>5))//�������� tracevalue��ʵ��ƽ������ֵ//P22�������� ����ʱ�����5S
	{
		if(tracevalue<=xram[22])//�������ڵ�������
		{
			l1=1;h1=0;//ʵ��������������ֵ
			minwarn=1;//�������ޱ���
			maxwarn=0;
		}
	}
	else
	{
		minwarn=0;
		l1=0;
	}
	
	////////ת�ӳӺ����¼� ////////  kg
	if((whtload>xram[23])&&(running>5))//�������� //�˲���ĺ���ƽ��ֵ��20�� //P23��������  ����ʱ�����5S   ����200
	{
		if(whtload>xram[23])//���ɳ������� 
		{
			h2=1;l2=0;
		}
	}
	else h2=0;
	if((whtload<xram[24])&&(running>5))//�������� //�˲���ĺ���ƽ��ֵ��20�� //P24��������  ����ʱ�����5S   ����5
	{
		if(whtload<xram[24])
		{
			l2=1;//���ɵ������� 
			h2=0;
		}
	}
	else    l2=0;
	////////����ƫ���¼���⼰����////////
	if((fabs(setvalue-tracevalue)>xram[37])&&(running>5)){h5=1;dout(4,1);}//����ƫ�Χ���� //P37����ƫ�Χ ƫ�����  �趨ֵ������
	else {h5=0;dout(4,0);}//ƫ�������
	////////�����趨ֵ�¼����////////
	if((!avmode)&&(setvalue<(xram[1]*0.02)))e5=1;//�ڷ��ݻ�ģʽ��//�趨ֵ���ڵ���  P1�������
	else e5=0;
	////////��������������¼����//////// �����ģ�
	if((addat<30||(addat>65532))&&(running>5))c1=1;//�������������  //24λ ����16λ 
	else  c1=0;//����
	////////�������////////if((speedcode<3)&&xram[9])speedless++;else speedless=0;//�ٶȴ�������Ч//�ٶȴ�������Ч  P9�ٶȴ������Ƿ�Ϊ��Ч�� 0�ٶȴ�������Ч���ٶ���P5������1�ٶ��ɴ���������	            	
	if((speedless>10)&&run&&(ctrlout>2000))  h7=1;//����10s������ʱ//speedcodeС��3 ����10s  //�ٶȸ���2000
	else  h7=0;	                               //����
	////////�ٶȴ������������////////
	if((speedcode>1500)&&xram[9]&&(running>5))  //����
	{
		if((speedcode>1500)&&xram[9])c3=1;//75*20 �ٶȴ������������
		else c3=0;
	}	
	////////�ٶ���������¼�///////
	if(((ctrlout/speedcal)>=xram[31])&&run)h6=1;//����������ֵ //������������  ���ݹ�ʽ��20.8845*B/4096 =�������  ����
	else h6=0;
	////////�����趨ֵ�������///////
	if(((setvaluetemp>xram[1]*3)&&avmode)||(!avmode&&(setvaluetemp>xram[1])))s9=1;//�趨ֵ��������//��������
	else s9=0;
	////////�ٶȴ��������///////
// 	if((speed_check==0)&&xram[9])c2=1;//��ȡ�ٶȼ��˿ڣ�����0 �ٶȱ���   //�ٶȴ������������
// 	else c2=0;
	////////Ԥ���ϻ�������ֹͣ///////  ���� �ֳ�����
	if(pre_feeder&&(run))dout(5,1);//Ԥ���ϻ�����   Ԥ���ϱ�־1    
	else dout(5,0);//Ԥ���ϻ��ر�	
	
/**********************������*******************/	
	if((dis_signal<1)||(dis_signal>10))dis_signal=2;//������ʾ����
	if((pre_feeder<0)||(pre_feeder>1))pre_feeder=0;
	if((keymode<0)||(keymode>1))keymode=0;//����ģʽ���λ
	if((avmode<0)||(avmode>1))avmode=0;
	if(jgdis<0)jgdis=0;    
 }		
}	  
/**********************�ٶȼ����ж�*******************/	//ת���ٶȿ���   //ֻ������ʱ�������źŲ��ܸ���
void TIM8_UP_IRQHandler  (void)   //TIM8�ж�  ת���ٶȵĿ�������
{
 if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)  //���TIM8�����жϷ������
 {  
	   u16 i,j,avt;//
	   u32 addata=0,count,a=0;
	   TIM_ClearITPendingBit(TIM8, TIM_IT_Update);  //���TIMx�����жϱ�־ //ע��˳���  ��������ܽ����ж�   ��ʱ��������һֱ�ڼ���
// 	 TIM_SetCounter(TIM8, 0);//����������
	 /////////////SysTick��ʱ�Ĵ������ݱ���//////// 
	   s_val=SysTick->VAL;//��Ϊ0 �� 16λ Ϊ0 Ϊ0 16λ����
	   if(s_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����1
	   s_ctrl=SysTick->CTRL;
// 	 cpu_led=~cpu_led;
	   plusmul++;  //���ٶȶ�ʱ��
	   avt=tare_s; //Ƥ�صĶ�Ӧֵ
     tare_s++;
  /**************************** AD7730��ֵ����*********************************///����16λ����
//if(Zero_flag==1)
//{
    ad_flag=1;//����AD�жϵĶ�ȡ
		for(i=0;i<26;i++)
		{
		  ad7730_code_result[i]=ad7730_filter[i];  //����ת��
		}
		qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//����������  ��С��������
		addata=0;
		for(i=5;i<21;i++)
		{
			addata+=ad7730_code_result[i];//5��20���  16���� 
		}
		addata>>=12;//����12λ������4096,������4λ������16������8λ����256//ȥ��8λ���� //����16����ȡ��ֵ��ƽ��ֵ  24λȥ��8λ���� 16λ
		addat=addata;//16λ��ֵ   //100ms����һ��
		ad_flag=0; //����AD�жϵĶ�ȡ
	  sumquality=addat*presspra-tare_date_result[avt];
		
//}	

	 //Ƥ������  ��ȥƤ��    //��Ҫ�޸�  kg ���ֳ��Ƿ���Ҫ�˲�����	 ��ֵ����������������  ��������   Ƥ�ش�������
	 /******************************����ģʽת��**************************/ 
	  plusecount++;//��λ��ת�������
// if(tare_flag1==1&&tareline==1)    //��Ƥ�궨���궨Ȧ������ //�����жϺ��ж��Ƿ����㿪ʼ����
// 		{ 
// 			j=tarecount;  //Ƥ�ش���
//      	a=addat;      
// 			tare_date[j]+=a;			 //����ֵ�ӵ����������Ӧ�ĵ�jλ
// 	    tarecount++;           //�������+1
//    }
	  if(modecount==1) //����  ����� ת�Ӱ��趨ת����ת
    { 
		 EXTI_Set_Int(1);
     plusesum+=plusecount*xram[63];//����ģʽ1����¼��ת���壬�ѻ��۵���ת��ȥ  xram[63]�ָ�������
		 if((plusesum>=2*xram[64])&&Zero_flag==1) //�ۼƽǶȴ�����Ч�Ƕ�//��ת��������������Ч�Ƕ�������  
		 {
			plusesum=0;  //��ת�������
			modecount=2;	//ģʽ����
		  plusecount=0;//�����жϵļ���
			EXTI_Set_Int(0);
     }
    }
	   else 
		 if(modecount==2) //ģʽ2ʱ�䵽ת����ģʽ3  //���忪����    
		 {
			 plusesum+=plusecount*Sepapulse;//��¼���ۼ�����
			 if(plusesum>=2*xram[64]) //��ת��������������Ч�Ƕ�������
			 {
				  plusesum=0;//�ۼ�����������
				  modecount=3;//ģʽ3
			    plusecount=0;//�����жϵļ���
			 }
		 }
	   /**********���Ʋ�����ȡ��q1������ȡ���������͵������//qn��ȡ���������Ϳ����ٶȣ�*************/
		if(modecount==2||modecount==3)
		 {    
			   
				 partmoment=0;
				 for(count=separa;count>1;count--) //1��seprara 
				 {  
					 Q_quality[count] =Q_quality[count-1];//��������	  �ָ���ת
				 } 				 
				 for(count=2;count<=separa;count++)//��������
				 {
					 partmoment+=Q_quality[count]*Q_lenth[count];  //���ȵ�λCM
				 }
				 Q_quality[1]=(sumquality*xram[70]-partmoment)/Q_lenth[1];//���q1  �����������  P70: L  ������*����=������*������  �п��ܳ��ָ��������ڼ����Ĳ�׼ȷ��
				 whtload =0;
				 for(count=1;count<=separa;count++) //ֻ�����е�ʱ�� ���и���  ������ʱ������ʾ�Ǹ�����  
				 {
              whtload+=Q_quality[count]; //�����ܸ���
         }
				 if(modecount==3) //����ģʽ3   �����¹�ʽ
				 {	 
				   if(Q_quality[separa]>0)
				  {
				   speed_set=(setvalue*angledive)/(Q_quality[separa]*3.6);	//��ø����ٶ�  setvalue=Q_quality[separa]/angledive*speed_set*3.6 =kg/��*��/s*3.6
				  } 
				 }  
			}
		 
	     /*******************************�����ļ��㴦��*******************************************/  //������ֵ����ȡ
	  if(modecount==1)  //������ʱ������ģʽ1  ���ϵĹ���
		  {
// 			 press=sumquality/(xram[64]-anglemode);//ģʽ1����������   ģʽ1ʣ��Ƕ�
			   press=0;//����������
			   Xn=0;//����������
		  }
		else if((modecount==2)||(modecount==3)) //2 3
		  {
			 	 press=Q_quality[separa]/angledive;//��λ����������  kg/��   //����������    //ÿ�����εĽǶ�angledive
			   Xn=Q_quality[1]/angledive;// kg/��   ���������   (�ָ��Ƕ�)  //��63 �ָ��Ƕ���������/��77 һȦ������������*360
		  }
			press=press_1+aq*(press-press_1);//AQ�����˲�  //�˲���ĺ���  //
			press_1=press;
			for(i=1;i<press_ak;i++)averpress[i-1]=averpress[i];//press_ak=20  ���������ƶ� 0-press_ak-1   20�� ��19
			averpress[press_ak-1]=press;
			pressvalue=0;  //
			for(i=0;i<press_ak;i++)pressvalue+=averpress[i];
			pressvalue/=press_ak;//�˲���ĺ���ȡ��ֵ   20����ֵ  //��ʾ�����ļ����õ��˱���  
			
		/************************************�ٶȵĿ���***************************************/ 
  if((run)&&(!calflg))//���в��ҷǱ궨ģʽ  
	{
			////////////////////������Ӧ��////////////////////
		 
			if(modecount==1)//����������ģʽ1 �ص��� ��ת���ٶ� ��ʣ�����©��ȥ��
			{
			    ctrlout=xram[39]*speedcal;//P39�ݻ���ʽ�������� 4096/22.9  �����ֵ��Ӧ�������� //����  �����ݻ���ʽ����ת�٣
      }
			else if(modecount==2)  //�������ת���ٶ�  �ָ���Ԫ��ʼ�Ƿָ���Ԫ������
			{   
          ctrlout=xram[39]*speedcal;//P39�ݻ���ʽ�������� 4096/20.88  �����ֵ��Ӧ�������� //����  �����ݻ���ʽ����ת�٣�         				
      }
      else if(modecount==3)  //�ݻ���ʽ���ֶ�ģʽ������Ŀ���  �ٶȵĿ��Ʒ��ڼ�������
			{   
				if(avmode||(vmode))//�ݻ���ʽ���������������У�������ʼʱ���Ȱ����¿���һ��ʱ��
				{  
				  ctrlout=xram[39]*speedcal;//P39�ݻ���ʽ�������� 4096/22.9  �����ֵ��Ӧ�������� //����   
					ctrlout+=avdanum;
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //�ݻ���ʽת��ת�ٵĿ���
			  }
// 		  else if((xram[38]==1)&&(h5==0)){}//ƫ�Χ�ڲ�����P38ƫ�����ѡ�񣺵�ѡ��0ʱ�����κ�ʱ���ڵ��ڵ���ٶȡ���ѡ��1ʱ����ƫ�Χ�ڲ����ڵ���ٶ�  ��ƫ�����ʱ
				else//����������ʽ                               //ѡ��˲����ɼ�С����𵴣�ʹ����������ƽ�ȡ� 
				{
				  if((bflag==0)||(((zb-zi)>0)&&bflag))//����������־λ0 bsetend 5*set_value/3600//�����趨ֵ  ((zb-zi)>bsetend)&&bflag)  zb���������趨ֵ zi������������
				  {   
					ctrlout=speed_set/xram[69]*analogpra1+analog1; //xram[68]=0//�ٶȶ�Ӧ��ֵ/�ٶ�=��λ�ٶ���ֵ
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //�ݻ���ʽת��ת�ٵĿ���
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //����������β����  ȥ��
					{
						if(bctr==0)
						{
							danum=ctrlout/220;
							bctr=1;
						}
						ctrlout-=danum;//100ms�Ŀ�������  �����ļ�С
						if(ctrlout<(damin+300)) ctrlout=damin+300;
						else if(ctrlout>damax) ctrlout=damax;
						if(ctrlout>4095) ctrlout=4095;
					}
				}
		 }
  }
		if(mainboardflag==0)//�������������У׼ʱ���˱�־λ��1 ���������
		{
		   DAC7612(0,ctrlout);
		}//�ٶȿ������       ��������һ������
 /////////////SysTick��ʱ�Ĵ������ݻ�ԭ////////
			SysTick->LOAD=s_val;
			SysTick->VAL=0x00;
			SysTick->CTRL=s_ctrl;
	 
  }

}

/*********************************��ʱ��3�Ƚ��ж�********************************/

//��ʱ��3�жϷ������
void TIM3_IRQHandler (void)//TIM3�ж�
{
 if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
 {            
	  float speed_value=0;
	  uint16_t i,DS3234_code;
	  uint32_t ls_sec,addatab;
	  TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx�����жϱ�־ //ע��˳���  ��������ܽ����ж�   ��ʱ��������һֱ�ڼ���
	  /////////////SysTick��ʱ�Ĵ������ݱ���//////// 
		t_val=SysTick->VAL;//��Ϊ0 �� 16λ Ϊ0 Ϊ0 16λ����
		if(t_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����1
		t_ctrl=SysTick->CTRL;
	   ///////////////////////////////////////////
		t_tmp++;//*
		flash_light=!flash_light; //ȡ��ָ��//���ϱ���ָʾ����
	   ///////////////////////////////////////////
	  if(run==0&&flashblag==1)  //��������ڸı����ʱ�� �ڳ�����ʱ �޸Ĳ���ʱִ��
	  {
        writepra();
	     flashblag=0;  
    }
    /////////////////////////////////�ٶȼ���///////////////////////////////////	
		if(xram[9])//�ٶȴ������Ƿ���Ч��1����Ч�������Ч����Чʱ���ɶ�ٶ�ȷ��
		{
			speedcode=plusmul*Sepapulse+TIM_GetCounter(TIM8)-pluscountup;//������������  P77:�ָ�������
			pluscountup=TIM_GetCounter(TIM8);//��¼���ζ�ʱ������ֵ
			plusmul=0;//��������
			speed_count++;//��ʱ���жϴ���
			speedcode_tal+=speedcode;//������� //ת�ӳ�speedpra����������ת����ת�ĽǶȣ�
// 			anglecount =speedpra*speedcode;//��¼��ת�Ƕ� 100msת�ӳ�ת�˶��ٽǶ�
			speed_value=10*speedpra*speedcode;//�����ٶȡ�speedpra���������ӦƤ�����߳���  Ϊʲô����10   ÿ100msƤ������x ,1s����10*x =speed_value 1SƤ�����ߵĳ���
			speedvalue=0.6*speedvalue+0.4*speed_value;//��Ȩ speedvalue��������ʱ�� 
			if(speed_count==10)//10����ƽ��  1s
			{
				speed_code=speedcode_tal/10.0;
				speed_v=10*speedpra*speed_code;//����10�ε�ƽ���ٶ�  M/s
				speed_dis=speed_v1+av*(speed_v-speed_v1);//ƽ���ٶ��˲�
				if(speed_dis<0)speed_dis=0; 
				speed_v1=speed_dis;//�ٶ�ֵ //������  put_disdat��������  //��dissignal(void)���� ����ʾ���� speed_v1û����
				speed_hz=speedcode_tal;//��������  ��ʱ���ж�10�ε���������put_disdat��������  �ٶ�Ƶ��  1s����������ֵ 
				speedcode_tal=0;
				speed_count=0;
			}
		}
		else{speedvalue=xram[5];}//*/ //��ٶ�  m/s  0.1  �������ö�ٶ�
		
	
// 		if(Timedata>=2) //300ms��ʼ��AD7730  //�����������ʼ��     //����
// 		{
// 			c4=1;//C4AD7730��������
// 			EXTI->IMR &= ~(EXTI_Line15);// �����ⲿ�ж�15;//�ر��ⲿ�ж�
// 			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//����3����ʹ�ܽ�ֹ
// 			
// 			AD_RST =0;
// 			delay_us(40);
// 			AD_RST =1;//1
// 			delay_us(40);
// 			Timedata=0;
// 			ad7730_init();
// 			EXTI->IMR |= EXTI_Line15;//ʹ���ⲿ�ж�15;//���ⲿ�ж� AD7730�ж�
// 			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//����3����ʹ�ܽ�ֹ//����3�����ж�ʹ�ܿ�
// 		}
		Timedata++;//*/ AD7730��ʼ����  ÿ200ms��ʼ��һ��
		DS3234_WRITESRAM_VALUE();//���ֶ��������sram��д
		/*****************************��ȡ��ǰʱ��*********************************/
		DS3234time[0].second=Read_data_DS3234(0x00); //second  0  
		DS3234time[0].minute=Read_data_DS3234(0x01); //minute
		DS3234time[0].hour=Read_data_DS3234(0x02);   //hour
		DS3234time[0].week=Read_data_DS3234(0x03);   //week
		DS3234time[0].date=Read_data_DS3234(0x04);   //date
		DS3234time[0].month=Read_data_DS3234(0x05);  //month
		DS3234time[0].year=Read_data_DS3234(0x06);   //year//*/
		/*****************************���¶���ֵ*********************************/
		if(temper_flag==1)  //��־λ�ڷ����������  ֻ��Ϊ1�Ŷ�ȡ�¶�  �ڷ���ֵ���õ�
		{
			DS3234_status_flag=Read_data_DS3234(0x0f);//BSY bit status
			if((DS3234_status_flag&0x04)==0x04){delay_us(3);}
			else
			{
				DS3234_code=Read_data_DS3234(0x11);//���ֽ�
				DS3234_code<<=8;
				DS3234_code+=Read_data_DS3234(0x12);//���ֽ�  
				DS3234_code>>=6;//����10λ�����λ����λ
				delay_us(1);
				if(DS3234_code<512)//2��9���� Ϊ�� 511
				{
					tempcode=DS3234_code;
					tempcode>>=1;//��С1λ����  /?
					temp_neg_flag=0;
				}
				else//Ϊ��
				{
					DS3234_code=(~DS3234_code);//ȥ�� 
					DS3234_code&=0x7f;
					DS3234_code+=1;//�Ǹ��� ȥ����1
					tempcode=DS3234_code;
					tempcode>>=1;
					disbuf[31]='-';
					temp_neg_flag=1;//������־λ
				}
			}
		}//*/
		/*****************************ʱ���޸�*********************************/
		if(Alter_time_flag==1) //�궨ʱ��
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
	   /*****************************Ϊʲô��*********************************/  //��ԭ������ʵ���ж�Ƕ��
// 	     USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//����3����ʹ��//����1�����ж�ʹ�ܿ�
// 	     EXTI->IMR |= EXTI_Line15;//ʹ���ⲿ�ж�15;//���ⲿ�ж� AD7730�ж�
// 	     SEI();//��ȫ���ж�
	   /*****************************ʱ���¼��־λ*********************************/
		 if(ms_secb>=10){ms_secb=0;} //��Ƥ�궨��ʱ  ��
	     ms_secb+=1; //ÿ100ms+1   2sʱ���� 
	   /**************************** ˢ����ʾ*********************************/
		 dis_play();
           /**************************** AD7730��ֵ����*********************************///����16λ����
		if(run==0)  //ת�ӳ�û������ʱ����Ҫ������ʾ���ɣ�û������ʱ�ĸ�����Ϊ��ֱ�Ӳ���-Ƥ�أ�û��ͨ���ܸ˱ȵ�ת��
		{		
			ad_flag=1;//����AD�жϵĶ�ȡ
			for(i=0;i<26;i++)
			{
				ad7730_code_result[i]=ad7730_filter[i];  //����ת��
			}
			qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//����������  ��С��������
			addatab=0;
			for(i=5;i<21;i++)
			{
				addatab+=ad7730_code_result[i];//5��20���  16���� 
			}
			addatab>>=12;//����12λ������4096,������4λ������16������8λ����256//ȥ��8λ���� //����16����ȡ��ֵ��ƽ��ֵ
			addat=addatab;//16λ��ֵ 
			ad_flag=0; //����AD�жϵĶ�ȡ
			sumquality=addat*presspra-tarevalue;//sumqualit Ϊ���ش�����ֱ�Ӳ�����û��ͨ���ܸ˱ȵ�ת��
	  }
		/**************************** �������ʱ���Ƿ��ѵ�,������˾��˳�*********************************/
		if(xram[125]==1)
	  {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //���TIMx�����жϱ�־ �������־λ���������ж�
		 return;//�������ʱ����λ  ��Ҫ ��Ҫ 
    }			
		/**************************** ��ʱ��־λ*********************************/
		if(hsec>=10)//1.0s   1sѭ��һ��
		{
			
			hsec=0;
			ms_secb=0; 
			tx0_sec=0;//dissignal��   
			sec=sec+1;//�ͱ궨�����й�
			if(rjtime)rjtime--;//�տ�ʼ����ʱ���ݻ�������ʽ
			else{vmode=0;}//      
			if(run)//����ٶȴ�������
			{
				if(running<200)running++; //1s���һ��   ���¼���Ϣ�ã�
				if((speedcode<3)&&xram[9])speedless++;//�ٶȴ�������Ч  P9�ٶȴ������Ƿ�Ϊ��Ч�� 0�ٶȴ�������Ч���ٶ���P5������1�ٶ��ɴ���������
				else speedless=0;//(�ٶȴ�������Ч)����(�ٶȴ�������Ч����speedcode>3)
			}
			else {speedless=0;running=0;}//��û������
			if(s5_flag);//������Ч��־λ  
			else if(s5)  //��ͨ����
			{
				s5_second--;
				if(!s5_second)s5=0;
			}
			if(s6_flag);
			else if(s6)  //��������
			{
				s6_second--;
				if(!s6_second)s6=0;
			}
		}
		/*********************************����1��ʱ��������*****************************/
		if(usart_send_time>=50)
		{
			usart_send_time=0;
			if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			{
		   switch(tx0_count)//���ͼ�����  //���� ʱ��  1���ۼ� 2���ۼ� 3���ۼ� ���ۼ�
			 {
				case 0:USART_SendData(USART1, prn_time[0]);tx0_count++;break;//��ӡʱ��  ��     
				case 1:USART_SendData(USART1, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART1, '-');tx0_count++;break;
				case 3:USART_SendData(USART1, prn_time[2]);tx0_count++;break;          //��
				case 4:USART_SendData(USART1, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART1, '-');tx0_count++;break;	
				case 6:USART_SendData(USART1, prn_time[4]);tx0_count++;break;          //��
				case 7:USART_SendData(USART1, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART1, 0x0d);tx0_count++;break; //�س�
				case 9:USART_SendData(USART1, 0x0a);tx0_count++;break; //����
				case 10:USART_SendData(USART1, prn_time[6]);tx0_count++;break;         //ʱ
				case 11:USART_SendData(USART1, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART1, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART1, prn_time[8]);tx0_count++;break;         //��
				case 14:USART_SendData(USART1, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART1, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART1, prn_time[10]);tx0_count++;break;        //��
				case 17:USART_SendData(USART1, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART1, 0x0d);tx0_count++;break;//�س� Enter
				case 19:USART_SendData(USART1, 0X0a);tx0_count++;break;//����
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
				case 42:USART_SendData(USART1, prnbuf1[7]);tx0_count++;break;//prnbuf1  1�����ۼ���
				case 43:USART_SendData(USART1, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART1, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART1, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART1, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART1, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART1, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART1, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART1, 0x0d);tx0_count++;break;//�س�����
				case 52:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART1, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART1, 'H');tx0_count++;break;
				case 55:USART_SendData(USART1, 'I');tx0_count++;break;
				case 56:USART_SendData(USART1, 'F');tx0_count++;break;
				case 57:USART_SendData(USART1, 'T');tx0_count++;break;
				case 58:USART_SendData(USART1, ':');tx0_count++;break;
				case 59:USART_SendData(USART1, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART1, 0x0d);tx0_count++;break;//�س�
				case 61:USART_SendData(USART1, 0x0a);tx0_count++;break;//����
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
				case 84:USART_SendData(USART1, prnbuf2[7]);tx0_count++;break;//prnbuf2  ����2
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
				case 126:USART_SendData(USART1, prnbuf3[7]);tx0_count++;break;//prnbuf3  ����3
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
				case 168:USART_SendData(USART1, prnbuf4[7]);tx0_count++;break;//prnbuf4 �ܵ��ۼ��� 
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
				case 193:USART_SendData(USART1, 0x0d);tx0_count++;break;//�س� Enter	
				case 194:USART_SendData(USART1, 0x0a);tx0_count++;break;//����
				case 195:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 196:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 197:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 198:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 199:tx0_count=0;break;
			}
		}
	}
		 /**************************** �����������ʾ���¼�������ʾ��*********************************/
		  if(t_tmp>=1) //++�ڶ�ʱ�жϿ�ʼ   ÿ100msִ��һ��
		  {
			t_tmp=0; //++�ڶ�ʱ�жϿ�ʼ
			tx0_sec++;//
			hsec++;
			usart_send_time++;
			if(disrunflg)//�ܶຯ�������˱�־λ
			{
				/////////////////�ݻ�����ģʽͼ����ʾ///////////////////
				if((avmode==1)&&(keymode==1))//���̺��ݻ�ģʽ 
				{ 
					disbuf[0]=0xa1;
					disbuf[1]=0xf8;
				}
				else if((avmode==0)&&(keymode==1))//����ģʽ
				{
					disbuf[0]=0xa1;
					disbuf[1]=0xf7;
				}
				if(keymode==0)//
				{
					disbuf[0]=0x20;//����ʾ
					disbuf[1]=0x20;
				}	
				
				ls_sec=(uint32_t)sec;
				i=ls_sec%2;//������
				
				/////////////////��������ͼ����ʾ///////////////////
				if(run&&i)//������״̬��secλ����ʱ
				{
					if(bflag==0) //û����������
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xfa;
					}
					else //��������
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xc6;
					}
				}
				else//secΪż�� ����û������ʱ
				{
					if(run&&bflag)//����������������־λ1
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
				
			  /////////////////���ݻ�ͼ����ʾ������������ʽ��������///////////////////
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
				
				/////////////////�����¼�ͼ����ʾ///////////////////
				if(warnflag&&!i)//������־λ1  ��˸��
				{
					disbuf[20]=0x20;
					disbuf[21]=0x20;
				}
				else
					warn();//�¼�������ʾ
			  }
		   }
		 /**************************** �ۼ��������*********************************/
		 if(pulsecount)
		  pulsecount--;  //��������ʱ�� //ʱ�����������pulsecount 
 	    else
			{
				dout(6,0);   //�ۼ��������
			}
	    /**************************** �����¼�����*********************************/

      if(c1||h7||h4||l4||((xram[41]==1)&&h9))//C1���ش������������//h7�������//h4���ش��������ع���//l4���ش���������̫С ����  ���ֹ���ʱ��������� plc�������źŹر� 
			{
				ready_flag=1;//����������  //�±���      //P41 Ƥ����ƫ����ѡ�� 1 ����X6��5��6�Ͽ�ʱ���Ǳ������������ֹͣ//h9Ƥ����ƫ���ֳ����� 
				if(flash_light==1)dout(10,1);//����ָʾ�Ʊ�����˸
				else dout(10,0);
				dout(2,1);//��������ź� DO3������� �̵����������
			}
			else//�����¼���û����ʱ��
			{
				ready_flag=0;
				dout(10,0);//��������ָʾ�ƹأ������źŹ�
				dout(2,0); //�̵����������
			}
			////////���׼�����///////
			if(!keymode)//�Ǽ���ģʽ  �¶����DO3Ϊ�������(Ӱ��ͣ���ı���) DO8����Ϊ����  �Ǽ���ģʽʱ�����ײ������PLC������PLC�����źŲ�������Ӷ��Ǳ��ֹͣ
			{
			///	if(!c1)dout(2,1);//�ؼ��̡���AD������������ס�
			///	else dout(2,0);
				if((xram[46]==0||xram[46]==2)&&(!ready_flag)) dout(8,1);//�ޱ���,����������ź� //P46=0 ����Ϊ������X3:3,4;�趨ֵֵ��ģ����X6��1��2  �޸Ĺ� p46=0��P46=2�������Ҫ����
				else dout(8,0);//����ʱ�����������
				if((xram[43]==1)&&(xram[46]==1)) status[0]=0x31;//P43=1 �д���ͨѶ =0�޴���ͨѶ p46=1 �������趨ֵ��Դ�ɴ������ �����������������Ϣ//����������
				else status[0]=0x30;
				
			}
			else//����ģʽ  //����ģʽ������������ź�
			{
			///	dout(2,0); 
				dout(8,0);//�����̹ر���
				status[0]=0x30; //�趨ֵ�ɼ������룬������ֹͣҲ���Լ���
			}
			if((dis_dissigal_flag==1)&&(menu_exit_flag==1)) //��ຯ���õ��� //��Щ��־λ����ĳЩ������ִ��ʱ��
			{
				dis_sec_count++;
				if(dis_sec_count>=2) //200ms
				{
					dis_dissigal_flag=0; //������ʾ������
					menu_exit_flag=0;
					dis_sec_count=0;
				}
			}
			if((xram[43]==1)&&(M485REI ==1)&&(M485DEI==1))//P43λ1 �д���ͨѶ //RE =1(PJ4),DE =1(PJ3) ��ʱ����ģʽ     //ʹ��RE=0 DE =1  485����3s,�������ǽ��� ��485���պͷ���Ƶ��Ϊ3s
			{
				RE_DE_Count++;
				if(RE_DE_Count==30)
				{
					USART_ITConfig(USART3, USART_IT_TC, DISABLE);//disable TX �رշ���        //�رշ�������ж�
					USART3->CR1|=1<<2;    //���ջ������ǿ��ж�ʹ��;		//enable RX  �򿪽��ܣ�����򿪣�������ر��˽��գ�   //�رյ��Ƿ���  
					USART_ClearFlag(USART3,USART_SR_RXNE);
					M485REI =0;			//RE enable ;DE disenable // RE =0  DE=0 /���Խ��ղ��ܷ���
                    M485DEI =0;					
				}
			}
			else RE_DE_Count=0;			
			/*************************************����ֹͣ����****************************************/
// 			if((!din(2))&&(run)&&(keymode)&&(xram[46]==0)) stop();//DI3�ͷſ��ش�//����//����ģʽ//P46=0:�������趨ֵ�����Զ˿� �������⣩���� �Ƿ���Ҫ�ж�P46 
			if((!din(2))&&(run)&&(keymode)) stop();//�Ĺ�������һ������
			if((!din(2))&&(din(1))&&(run)&&(!keymode)&&((xram[46]==0)||(xram[46]==2))) stop(); //run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0;  DI3  DI2
			if((run)&&((c1)||(c2)))
			 stop();//����////���ش������������////�ٶȴ������������   ����ʱ��ͣ�� ����
			/*************************************�Ǽ���ģʽ��ͣ���ƣ���ͣ�˿ڿ��ƣ�****************************************/
			if((!run)&&din(1)&&din(2)&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//�ر�//DI2 ������ֹͣ//DI3�պ�//ֹͣ����ģʽ//�����ɶ˿ڣ��趨ֵ�ɼ��̻��Ƕ˿�   DI3  DI2
			{
				start();//���������������4-20mA���   // 1.�Ǽ���ģʽ  �������Զ˿�   
			}
			if((run)&&(!din(1))&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//����//DI2������ֹͣ//ֹͣ����ģʽ//�����ɶ˿ڣ��趨ֵ�ɼ��̻��Ƕ˿�  ���˳�ֹͣ��
			{
				stop();//���������������4-20mA���   
			}
			/***************************************����ģʽֹͣ����*******************************************/
			  if(keymode==1)//����ģʽ
			 {   mhang0=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_0); //��ȡ��
				 mhang1=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_1); 
				 mhang2=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_2); 
				 hang_1=1;//
				 hang_2=1;//
				if(run)//���������״̬
				{
					hang_0=0;
					if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)
					{
						delay_us(10);
						if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)stop();
					}
				}
				else//��ֹͣģʽʱ
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
        /**************************** CPU�������ֵ�Ĵ���*********************************/
			anainput=flow_set();//�����  ������ֵ   ����ʽ����
      if((!keymode)&&(xram[46]==0))//ϵͳ����ֹͣ���̷�ʽ��0 �趨ֵ����Դ��ģ����X6��1��2����    ����Ϊ������X3��3��4   //CPU������趨ֵ�����Ĵ���
			{
				lsvalue=(anainput-settare)*setpra*input_rate;//input_rate �ֶ�����//������Ӧ�Ķ��� setpra��λ��ֵ��Ӧ��������t/h�� input_rate(�ֶ��������ֵ)
				if(lsvalue>0)//����ֵ�˲�
				{
					setvalue=setvalue_1+ap*(lsvalue-setvalue_1);//ap=0.5/P54  P54���������˲�ʱ��
					setvalue_1=setvalue;
				}
				else setvalue=0;
			}
			/*************************************��������������ͣ����****************************************/
			if((xram[43]==1)&&(xram[46]==1)&&(!keymode)&&(r_flag=='Y'))//P43�д���ͨ��//�������趨ֵ�����Դ�������//ֹͣ����  //������
			{
				if((lsset=ser_setvalue())!=-1)//���ڸ���
					setvalue=lsset;//������������
				if((rs_temp[3]=='K')||(rs_temp[3]=='k'))
				{
					if((rs_temp[2]==0x31)&&(run==0)) //2 ������ֹͣ  //2.�Ǽ���ģʽ ��ͣ���Դ���
						start();
					else if((rs_temp[2]==0x30)&&(run==1))
						stop();
				}
			}
	
    /*******************************�����ļ��㴦��*******************************************/  //������ֵ����ȡ

			if(run)//����״̬�£���������ֵ
			{   
				ctrealvalue=press*speedvalue*3.6*xram[36];//t/h   �����ۼ���
				realvalue=pressvalue*speedvalue*3.6*xram[36];//pressvalue�غ� //p36ʵ��궨����ϵ����p36=p36*ʵ���ۼ�ֵ/��ʾ�ۼ�ֵ  ����ֵ��Ϊ���±�����ֵ�� kg/��* ��/s *3.6 =t/h(��������ֵ)   ��ʾ��
				lsreal=Xn*speedvalue*3.6*xram[36];  //p36ʵ��궨����ϵ����p36=p36*ʵ���ۼ�ֵ/��ʾ�ۼ�ֵ  ��������ƣ� kg/��* ��/s *3.6 =t/h ����������ֵ���Ƶ��壩
			}
			else {ctrealvalue=0,realvalue=0,lsreal=0;}//ֹͣʱ��������
			
			////////////////////��ʾ��////////////////////
			real=real_1+ai*(realvalue-real_1);//����ֵ�˲�
			real_1=real;
			for(i=1;i<20;i++)avernum[i-1]=avernum[i]; //��������
			avernum[20-1]=real;//ak=30
			tracevalue=0;
			for(i=0;i<20;i++)tracevalue+=avernum[i];//��ȡƽ��ֵ
			tracevalue/=20;//��ʾ�� ��������
			delay_us(1);

     /*******************************������Ƶ���*******************************************/ 
     if((run)&&(!calflg))//���в��ҷǱ궨ģʽ  
    {
			////////////////////������Ӧ��////////////////////
	  runflag_count++;
	  if(runflag_count>=5)runflag_count=5;
	  dout(7,1);//������   �ӵ������ź� ת�ӳӵ������
	  if(!xram[9]){speed_dis=xram[5];}//�ٶȴ�������Ч���ٶ���P5�����ϱ��Ѹ�speedvalue���ڼ��㣬speed_dis������ʾ //����
	  if(xram[9]){if(speedcode>5)dout(0,1);}//1�ٶȴ�������Ч��speedcode 100ms�ٶȼ�������ֵ   DO1=1 ����״̬  500ms�����ٶ�֮��(Ӧ���ź������   ����
	  else {if(runflag_count>=5)dout(0,1);} //Ӧ���ź����   //����
	  if(modecount==1)//����������ģʽ1 �ص��� ��ת���ٶ� ��ʣ�����©��ȥ��
	  {
       ctrloutb=daminb;//�ص���
    }
	  else if(modecount==2)  //�������ת���ٶ�  �ָ���Ԫ��ʼ�Ƿָ���Ԫ������
	  {   
      ctrloutb=xram[73]*boardcal;//P73�ݻ���ʽ�������� 4096/20.88  �����ֵ��Ӧ��������//����   �����ݻ���ʽ���Ƶ��忪�ȣ�         				
    }
    else if(modecount==3)  //�ݻ���ʽ���ֶ�ģʽ������Ŀ���  �ٶȵĿ��Ʒ��ڼ�������
			{   
				if(avmode||(!avmode))//�ݻ���ʽ���������������У�������ʼʱ���Ȱ�һ�¿���һ��ʱ��
				{  
					ctrloutb=xram[73]*boardcal;//P73�ݻ���ʽ�������� 4096/22.9  �����ֵ��Ӧ��������//����   
					ctrloutb+=avdanumb;
					if(ctrloutb<=daminb) ctrloutb=daminb;
					if(ctrloutb>=damaxb) ctrloutb=damaxb;
					if(ctrloutb>4095)    ctrloutb=4095;   //�ݻ���ʽ���忪�ȿ���
				}
				else if((xram[38]==1)&&(h5==0)){}//ƫ�Χ�ڲ�����P38ƫ�����ѡ�񣺵�ѡ��0ʱ�����κ�ʱ���ڵ��ڵ���ٶȡ���ѡ��1ʱ����ƫ�Χ�ڲ����ڵ���ٶ�  ��ƫ�����ʱ
				else//����������ʽ                               //ѡ��˲����ɼ�С����𵴣�ʹ����������ƽ�ȡ� 
				{
	////			ek1=ek;
					if((bflag==0)||(((zb-zi)>0)&&bflag))//����������־λ0 bsetend 5*set_value/3600//�����趨ֵ  ((zb-zi)>bsetend)&&bflag)  zb���������趨ֵ zi������������
					{     
						ek=setvalue-lsreal;//�趨ֵ-����ת�ӳӷ���ֵ=ƫ�� 
	////			danum=kpi*(ek*ki-ek1);//ek1��һʱ�̵�ƫ�� B/t/h*(t/h-t/h) =B  �������Ƴ��� ki =1+1/P12  Ƥ����
						danumb=(kp+ki+kd)*ek+(-kp-2*kd)*ek1+kd*ek2;//��������
						ctrloutb+=danumb;
						if(ctrloutb<=daminb)ctrloutb=daminb;//��ֵ��Сֵ
						if(ctrloutb>=damaxb)ctrloutb=damaxb;//��ֵ���ֵ
						if(ctrloutb>4095)ctrloutb=4095;
						ek2=ek1;
						ek1=ek;
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //����������β����  ȥ��
					{
						if(bctr==0)
						{
							danumb=ctrloutb/220;
							bctr=1;
						}
						ctrloutb-=danumb;//100ms�Ŀ�������  �����ļ�С
						if(ctrloutb<(daminb+300)) ctrloutb=daminb+300;
						else if(ctrloutb>damaxb) ctrloutb=damaxb;
						if(ctrloutb>4095)ctrloutb=4095;
					}
			 }
			 
		 }
  }
		else if(!calflg)//if((run==0)&&(!calflg)) //ֹͣ����
		{
			runflag_count=0;
			dout(0,0);//����״̬�أ�Ӧ��
			dout(7,0);//ת�ӳӹر� //����  ����
			ctrlout=ctrlana1;////ֹͣ�����СֵmA
			ctrloutb=ctrlana2;//ֹͣ���������СֵmA
			if(!xram[9]){speedvalue=0;speed_dis=0;}//�ٶȿ�����Чʱ
			plusesum=0;//�ۼ�����������
		    modecount=0;//ģʽ0 û����
			plusecount=0;//�����жϵļ���
		}
	/***************************************100ms������ȡ*******************************************/
		if(calflg)//�궨���  //�ݻ���ʽ����
		{
			total_temp=0;                                                       //P42 0:����������L2����ʱ���ۼƼ����������ۼ�//1:����������L2����ʱ���ۼ���������
			if((pressvalue<xram[24])&&(xram[42]==1)){delay_us(1);} //����С�ں��������뵱��������L2����ʱ���ۼƼ��������ۼ�
			else total_cal+=ctrealvalue*0.000027777778*xram[25];//���ۼ���//ÿ100ms�ۼ�һ����ֵ//p25 =p25(ԭֵ)*����ʵ��ֵ/��ʾ�ۼ�ֵ    ע������ʾ�ۼ�ֵ�������ۼ�ֵ  0.000027 =1/36000
		}
		else if((l2==1)&&(xram[42]==1))//����С�ں��������뵱��������L2����ʱ���ۼ�������  //1�ۼ������ۻ�
			{total_temp=0;}  
		else
			{total_temp=ctrealvalue*0.000027777778*xram[25];} //p25 =p25(ԭֵ)*����ʵ��ֵ/��ʾ�ۼ�ֵע������ʾ�ۼ�ֵ�������ۼ�ֵ��������//1h=3600s=36000��100ms��

/***************************************�ۼ���*******************************************/
	    z1_temp+=total_temp;//total_temp 100ms���ۼ�ֵ//total_temp����ֵ t  //�ۼƵ�����ֵ
// 			z1 =0;
// 			z1_temp =0;
// 			totalshift[0]=0;
// 			totalshift[1]=0;
// 			totalshift[2]=0;
			
		if(fabs(z1_temp)>=xram[26])//���������嵥λ
		{
			if(z1_temp>0)
			{
				z1+=xram[26];//���������嵥λ��λ����
				totalshift[shift]+=xram[26];//���������嵥λ ���ۼ�  �����ۼ�
				z1_temp-=xram[26];
			}
			else//ʲô����³���
			{
				z1-=xram[26];
				totalshift[shift]-=xram[26];//���������嵥λ ���ۼ�  
				z1_temp+=xram[26];		
			}
			if(npulse)dout(6,1);//�ۼ���//npulse=(uchar)(pramc[27]/100+0.9);//�����ۼ����������ʱ��
			pulsecount=npulse;	//ʱ�䵽 dout(6,0)	
		}
		if((bflag==1)&&run)//�����ɲ���־λ  //�����ɲ����ֹͣ
		{
			zi+=total_temp;//zi =0  ����
			zd-=total_temp;//zd =0  ����
			if(zi>=zb) //���������趨ֵ �����ֹͣ
			stop();	
		}

		if(clearflag1)//�ܶ�λ����
		{
			totalshift[shift]+=z1_temp;
			z1=0;//���ۼ���
			z1_temp=0;//�ۼƻ�����
			clearflag1=0;

		}
		if(z1>=9999999){z1-=9999999;}//

    //////////////////////////////////////////////////////////////////////
	if(!calflg){analogout();}//û���ڱ궨����ʱִ��  ��ǰģ�ⷴ�����  //ִ�б궨ʱ��û��ִ��
	
	if(mainboardflagb==0)//������Ƶ���
    {
	   DAC7612_2(0,ctrloutb);
    }//�������
	 
  if(mainboardflag==0&&deputyboardflag==0)//�������������У׼ʱ���˱�־λ��1 ���������
   {
	    DAC7612(0,ctrlout);
   }//�ٶȿ������   
	change_shift();//���� ȷ���Ǹ��� 
// 	cli();//���ж�                                  ������  //����
// 	TIMSK1=(1<<OCIE1A);//bit1 ����Ƚ��ж�ʹ��λ��   ������
	
	 /////////////SysTick��ʱ�Ĵ������ݻ�ԭ////////
	  SysTick->LOAD=t_val;
	  SysTick->VAL=0x00;
	  SysTick->CTRL=t_ctrl;

   } 	 	
	   
} 	 	


/*********************************�ⲿ�жϣ�AD7730����********************************/
void EXTI15_10_IRQHandler(void)
{
// 	delay_ms(10);//����
   if( EXTI_GetITStatus( EXTI_Line15)!=RESET)
   { 
		uint8_t i=0;
		Timedata=0;
		c4=0;
		/////////////SysTick��ʱ�Ĵ������ݱ���////////   
		e_val=SysTick->VAL;//��Ϊ0 �� 16λ Ϊ0 Ϊ0 16λ����
		if(e_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����1
		e_ctrl=SysTick->CTRL;  
			 
		if(ad_flag==0)//ad��־λ
		{
				if(INT7_flag==0)//�жϱ�־λ
				{
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//����������λ//���λ���鱻����
					buf7730=AD7730_READ();//������ֵ
					ad7730_filter[25]=buf7730;//�����˲���
				}
				else
				{
					INT7_flag=0;
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//����������λ
					ad7730_filter[25]=buf7730;//��һ�γ�����ֵ
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//����������λ
					buf7730=AD7730_READ();
					ad7730_filter[25]=buf7730;
				}
		}
		else//�ڼ���ˢ�³���ʱ��һֱ�����´δ��룬ֻ�ܱ���һ�β���ʧ
		{
					INT7_flag=1;
					buf7730=AD7730_READ();
		}	
	/////////////SysTick��ʱ�Ĵ������ݻ�ԭ////////
		 SysTick->LOAD=e_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e_ctrl;
	
		 EXTI_ClearITPendingBit(EXTI_Line15);  //���LINE15�ϵ��жϱ�־λ
  }	   
}

/*********************************����1�жϷ�����򣨽��ա����ͣ�********************************/
//void USART1_IRQHandler(void)
//{
//	u8 TEXT_TO_SEND[]={"tongxunxieyichengong"};
//	
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//���ָ����USART�жϷ������	
//	{
//		  uint8_t i=0,j=0;
//		  for(i=14;i>0;i--)//16������ 
//	  	{USART_RX_BUF[i]=USART_RX_BUF[i-1];}//���������ƶ� 
//		  USART_RX_BUF[0]=USART_ReceiveData(USART1); //����
//    	if(((USART_RX_BUF[14]=='F')||(USART_RX_BUF[14]=='f'))&&((USART_RX_BUF[10]=='T')||(USART_RX_BUF[10]=='t'))&&((USART_RX_BUF[2]=='N')||(USART_RX_BUF[2]=='n')))//ÿһ�����ݽ������  
//			{
//			  USART1->CR1|=0x0008;//ʹ�ܷ���
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
/*********************************����3�жϷ�����򣨽��ա����ͣ�********************************/
void USART3_IRQHandler(void)                	//����3�жϷ������
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
  {
	  uint8_t i=0;
	  float series_ls=0;
		for(i=11;i>0;i--)//12������ 0:ǰ��ȫ�����ݶ������ۼӺ� 1���Ǳ��ַ(16����) ��0,1���Ƕ������룩 2��0 1 ������ֹͣ����
		{rs_temp[i]=rs_temp[i-1];}//���������ƶ�  //3:"K" 4-10:7λ�����趨ֵ   11:"S"  
		rs_temp[0]=USART_ReceiveData(USART3);//(USART3->DR);//����
		if(((rs_temp[11]=='S')||(rs_temp[11]=='s'))&&((rs_temp[3]=='K')||(rs_temp[3]=='k')))//ÿһ�����ݽ������  
		{
			if(rs_temp[1]!=xram[44])return;//P44����ͨѶ��ַ����һ�������У�ÿ̨�Ǳ���ΨһͨѶ��ַ  ͨѶ��ַ���� �˳�  //����������
			r_crc=0;
			for(i=1;i<=11;i++)
			{
				r_crc+=rs_temp[i];//1-11���
			}
			if(r_crc==rs_temp[0])//��У��ɹ�
			{
			  r_flag='Y';t_crc=0;
				addr=rs_temp[1];//�Ǳ��ַ
				//////////////////=============/////////////////////////
				//////////////////���������ݴ���////////////////////////
				//////////////////=============/////////////////////////
				ts_temp[0]='S';
				series_ls=tracevalue;//�˲���ֵ����������ֵ
				put_series_data(series_ls,xram[0]);//����  С��λ��  //7λ���ݰ���С����
				for(i=0;i<7;i++)ts_temp[1+i]=tra_temp[i];//����ֵ7λ������С���㣩 ռ7������ װ��1��7����   ts_temp[1]Ϊ���λ
				ts_temp[8]='=';ts_temp[9]='P';
				series_ls=z1+z1_temp;//���ۼ��� 
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);//P34//�ۼ���С��λ��
				for(i=0;i<7;i++)ts_temp[10+i]=tra_temp[i];//10-16
				ts_temp[17]='=';ts_temp[18]='B';
// 				if(fser==0)                              //��
				series_ls=totalshift[shift]+z1_temp;//���ۼ�
// 				else//fser=1 ����ʱ��
// 				{
// 					if(shift==0)
// 				  series_ls=totalshift[shift+2];
// 					else
// 					series_ls=totalshift[shift-1];//
// 					lsfser=1;//����û����
// 				}
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);
				for(i=0;i<7;i++)ts_temp[19+i]=tra_temp[i];//19-25  ���ۼ�
				
				ts_temp[26]='=';ts_temp[27]='V';//
				series_ls=speedvalue;//�ٶ�ֵ
				put_series_data(series_ls,xram[4]);	
				for(i=0;i<7;i++)ts_temp[28+i]=tra_temp[i];//28-34
				
				ts_temp[35]='=';ts_temp[36]='W';//����
				series_ls=pressvalue;
				put_series_data(series_ls,xram[2]);	
				for(i=0;i<7;i++)ts_temp[37+i]=tra_temp[i];//37-43
				
				ts_temp[44]='=';ts_temp[45]='K';
				if(run)//�����ź�  status[0]�Ǳ��ף�  status[5]��status[6]�ǰ��ۼ�  //�¼���Ϣ���ڷ���  ����ACSII
					status[1]=0x31;//��ͣ״̬
				else
					status[1]=0x30;
				if(h9)		//��ƫ���ֳ�����
					status[2]=0x31;
				else
					status[2]=0x30;
				if((h2)||(l2))//Ƥ��ʵ�ʸ��س����趨ֵ����
					status[3]=0x31;
				else
					status[3]=0x30;
				if(h5)//ƫ���
					status[4]=0x31;
				else
					status[4]=0x30;
				for(i=0;i<7;i++)tra_temp[i]=status[i];
				for(i=0;i<7;i++)ts_temp[46+i]=tra_temp[i];//46-52
				
				ts_temp[53]='=';ts_temp[54]='C';
				ts_temp[55]=r_flag;//��һ�ν�����ȷ����Y��������N
				ts_temp[56]=xram[44];//P44��������ַ
// 				ts_temp[56]='1';
				for(i=0;i<4;i++)ts_temp[i+57]=' ';//����4���ո� 57-60
				for(i=0;i<61;i++)t_crc+=ts_temp[i];
				t_crc=t_crc+0x24;//0x24 =$  
				ts_temp[61]=t_crc;
// 				ts_temp[61]='A';
				//////////////////=============/////////////////////////
				//////////////////=============/////////////////////////
// 				UCSR1B|=(1<<TXCIE1);//���ͽ����ж�ʹ��
// 				UCSR1A&=0xbf;//���ͽ�����־λ����
// 				PORTJ|=0x18;//DE =1 RE =1 ʹ��485���ͣ��ر�485����
// 				UDR1=0x24;//����ǰ  //��������  $
				M485DEI=1;M485REI=1;//DE =1 RE =1 ʹ��485���ͣ��ر�485����
				USART3->CR1|=0x0008;//ʹ�ܷ���
				USART_ClearFlag(USART3, USART_FLAG_TC);
				USART_ITConfig(USART3, USART_IT_TC, ENABLE);return; //�����ж�ʹ�� 
			}
			else
			{
				r_crc=0;//����У�����
				r_flag='N';//����У��ʧ��	
			}
		}
	}
			
 
// 	 USART_ITConfig(USART1, USART_IT_TC, DISABLE);  
 //  USART_SendData(USART1,usar1_temp[ab] );	
//   USART_ClearFlag(USART1, USART_FLAG_TC);

// 		UCSR1A&=0xbf;////���ͽ�����־λ����	
 if( USART_GetITStatus(USART3, USART_IT_TC) == SET  ) //��������ж�   ���͸���30s��ʱ�䣬ʱ�䵽��ϵͳ���ڽ���״̬
{ 
    USART_SendData(USART3,0X24);       
	if((r_flag!=0x11)&&(addr==xram[44]))//����ɹ���ŷ��ͣ�r_flag ="Y"
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
			case 61:USART_SendData(USART3, ts_temp[61]);sart++;break;//У���
      case 62:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//���ڷ���ʹ�ܹر�  DE=0 RE=0  485 �رշ��� ʹ�ܽ���
 			default:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//���ڷ���ʹ�ܹر�  DE=0 RE=0  485 �رշ��� ʹ�ܽ���
		}
	}
	
  }

} 

/*********************************����2�жϷ�����򣨷�������жϣ�********************************/
 void USART2_IRQHandler(void)   
{
  if( USART_GetITStatus(USART2, USART_IT_TC) == SET  ) //��������ж�
  {
		if(xram[57]==1) //��ӡѡ��
		{
			switch(tx0_count)//���ͼ�����  //���� ʱ��  1���ۼ� 2���ۼ� 3���ۼ� ���ۼ�
			{
				case 0:USART_SendData(USART2, prn_time[0]);tx0_count++;break;//��ӡʱ��  ��     
				case 1:USART_SendData(USART2, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART2, '-');tx0_count++;break;
				case 3:USART_SendData(USART2, prn_time[2]);tx0_count++;break;          //��
				case 4:USART_SendData(USART2, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART2, '-');tx0_count++;break;	
				case 6:USART_SendData(USART2, prn_time[4]);tx0_count++;break;          //��
				case 7:USART_SendData(USART2, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART2, 0x0d);tx0_count++;break; //�س�
				case 9:USART_SendData(USART2, 0x0a);tx0_count++;break; //����
				case 10:USART_SendData(USART2, prn_time[6]);tx0_count++;break;         //ʱ
				case 11:USART_SendData(USART2, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART2, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART2, prn_time[8]);tx0_count++;break;         //��
				case 14:USART_SendData(USART2, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART2, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART2, prn_time[10]);tx0_count++;break;        //��
				case 17:USART_SendData(USART2, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART2, 0x0d);tx0_count++;break;//�س� Enter
				case 19:USART_SendData(USART2, 0X0a);tx0_count++;break;//����
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
				case 42:USART_SendData(USART2, prnbuf1[7]);tx0_count++;break;//prnbuf1  1�����ۼ���
				case 43:USART_SendData(USART2, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART2, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART2, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART2, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART2, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART2, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART2, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART2, 0x0d);tx0_count++;break;//�س�����
				case 52:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART2, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART2, 'H');tx0_count++;break;
				case 55:USART_SendData(USART2, 'I');tx0_count++;break;
				case 56:USART_SendData(USART2, 'F');tx0_count++;break;
				case 57:USART_SendData(USART2, 'T');tx0_count++;break;
				case 58:USART_SendData(USART2, ':');tx0_count++;break;
				case 59:USART_SendData(USART2, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART2, 0x0d);tx0_count++;break;//�س�
				case 61:USART_SendData(USART2, 0x0a);tx0_count++;break;//����
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
				case 84:USART_SendData(USART2, prnbuf2[7]);tx0_count++;break;//prnbuf2  ����2
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
				case 126:USART_SendData(USART2, prnbuf3[7]);tx0_count++;break;//prnbuf3  ����3
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
				case 168:USART_SendData(USART2, prnbuf4[7]);tx0_count++;break;//prnbuf4 �ܵ��ۼ��� 
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
				case 193:USART_SendData(USART2, 0x0d);tx0_count++;break;//�س� Enter	
				case 194:USART_SendData(USART2, 0x0a);tx0_count++;break;//����
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

void EXTI9_5_IRQHandler(void) //�궨Ȧ������
{
 if( EXTI_GetITStatus( EXTI_Line5)!=RESET)
 { 
		/////////////SysTick��ʱ�Ĵ������ݱ���////////   
		e5_val=SysTick->VAL;//��Ϊ0 �� 16λ Ϊ0 Ϊ0 16λ����
		if(e5_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//�رռ�����1
		e5_ctrl=SysTick->CTRL;  
	  EXTI_ClearITPendingBit(EXTI_Line5);  //���LINE15�ϵ��жϱ�־λ
//	EXTI_Set_Int(0);//���ⲿ�ж�ֹͣ��Ȧ��   
// 	  tare_s=0;
 if(calflg==1)
 {
   time_flag=1;
   EXTI_Set_Int(0);//���ⲿ�ж�ֹͣ��Ȧ��
  if(tare_cycle_flag==1)
  {
	  if(cycle_flag==1)//��һ�ν����жϿ�������1 ��������
		{ 
			TIM_SetCounter(TIM1, 0);//����������
			TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1 //TIM1��ʱ��ʹ��	 �궨ʱȦ�������ſ�	
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
        EXTI_Set_Int(0);//���ⲿ�ж�ֹͣ��Ȧ��
		    cycle_flag=1;
	      TIM_Cmd(TIM1, DISABLE);//�ض�ʱ��
      }
  }
  else//tare_cycle_flag=0
  {
// 	  tareline=1;   //�����жϺ��ж��Ƿ����㿪ʼ����
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
      EXTI_Set_Int(0);//���ⲿ�ж�ֹͣ��Ȧ��
	    tare_flag=1;
			TIM_Cmd(TIM1, DISABLE);//�ض�ʱ��
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
//       EXTI_Set_Int(0);//���ⲿ�ж�ֹͣ��Ȧ��
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
 
	/////////////SysTick��ʱ�Ĵ������ݻ�ԭ////////
		 SysTick->LOAD=e5_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e5_ctrl;	
 }	   
}




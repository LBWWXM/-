#include "string.h"//strcpyº¯ÊıÓÃ
#include  "stdio.h" //sprintfº¯Êı
#include "definition.h"
#include "config.h"
#include "drivers.h"
#include "sub.h"
#include "delay.h"
#include "math.h"
#include "stdlib.h"


extern char disbuf[40];//ÏÔÊ¾»º´æµ¥Ôª
extern u32 ad7730_filter[26];
extern u32 ad7730_code_result[26];
/*************************************************Ö÷ÎÄ¼ş±äÁ¿µÄ¶¨Òå*************************************************************/

char chartemp[10][23];

volatile float setvalue=0,z1=0,z1_temp=0,z2=0,z2_temp=0,z3=0,z3_temp=0,z=0,z_temp=0,input_rate=1,sec=0,speed_v1=0,speed_code=0,
				servicepressvalue0=0,servicepressvalue1=0,tracevalue=0,pressvalue=0,speedvalue=0,avervalue=0,speedpra=0,zb=0,zd=0,zi=0,
				nload=1,tarevalue=0,realvalue=0,ctrealvalue,total_temp=0,total_cal=0,bsetend=0,ai=0,av=0,ap=0,ac=0,ao=0,setvaluetemp=0,lsreal=0, 
				ctrlana1=0,ctrlana2=0,presspra=1,danum=0,danumb=0,ek=0,ek1=0,ek2=0,ki=0,kpi=0,kp=0,kd=0,kpv=0,aq=0,ctrlout=0,ctrloutb=0,devval=0,vspeedpra=0,stoppressvalue=0,setpra=0,
				settare=0,z1clear=0,z2clear=0,z3clear=0,kjtotal=0,kj_temp=0,G=0,Xn,press=0,press_1=0,ms_secb,averpress[press_ak],real_1=0,Sepapulse=0,
				real=0,setvalue_1=0,analogpra=0,analog0=0,analog1=0,analogpra1=0,analog2=0,analogpra2=0,totalshift_temp[3]={0},avernum[20]={0},
                totalshift[3]={0},speed_dis=0,speed_v=0,speed_hz=0,lsvalue=0,lsset=0,lastday_total={0},sumquality=0,boardset=0,partmoment,angledive,
                speedcal,flowcal,boardcal,whtload,plusecount=0,speed_set,plusesum,tare_date[TARE_NUM];
volatile uint8_t warnflag=0,s2=0,s3=0,s4=0,s5=0,s6=0,s7=0,s9=0,c1=0,c2=0,c3=0,s5_flag=0,s6_flag=0,run=0,ms_sec=0,dis_dissigal_flag=0,tare_flag=0,//Ô­s8±äs9
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
volatile int16_t 	damin=0,damax=0,daminb=0,damaxb=0,avdanum=0,avdanumb;//×ª×ÓËÙ¶ÈÂëÖµ  µ²°åÂëÖµµ÷½Ú
volatile uint32_t  	taretime=0,totalpulse=0,ontime=0,runtime=0,day_flag=0,addat=0,servicetime=0,buf7730=0,cycplussum1=0,cycss=0;
volatile u16 flow_data[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};   //0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//CPUÍâ¸ø¶¨ÊäÈëÂË²¨µ¥Ôª
volatile float xram[PRA_NUM],tare_date_result[TARE_NUM];//SRAM //0-128     
volatile float Q_quality[200],Q_lenth[200];//×ª×Ó³Ó 
union SRAMDS3234  DS3234SRAM;//	DS3234 SPAM´æ±äÁ¿
struct time DS3234time[2];//DS3234 Ê±¼ä²ÎÊıÖµ
 u8 mhang0, mhang1, mhang2;
//////////////////////////////////////////ÖĞ¶Ï±äÁ¿¶¨Òå/////////////////////////////////////////////
u32 t_val,t_ctrl,e_val,e_ctrl,s_val,s_ctrl,e5_val,e5_ctrl;   //¶¨Ê±Æ÷//Íâ²¿ÖĞ¶Ï//ËÙ¶È¼ÆÊıÆ÷ÖĞ¶Ï

					
int main(void)
{
   uint8_t i=0;
   CLI();//¹ØÖĞ¶Ï
// delay_ms (1000);
   config();//ËùÓĞ×ÊÔ´µÄÍâÉèÅäÖÃ IOÅäÖÃ ¶¨Ê±Æ÷ ¼ÆÊıÆ÷ÅäÖÃ
   d_out=1;//¿ª¹ØÁ¿Êä³ö×´Ì¬Î» 1Ê±ÔÊĞíÊä³ö
   dout(0,0);dout(1,0);dout(2,0);dout(3,0);dout(4,0);dout(5,0);dout(6,0);dout(7,0);dout(8,0);//¿ª¹ØÁ¿¸´Î»
   delay_ms (500);
   d_out=0;//¹Ø±Õ¿ª¹ØÁ¿Êä³ö
   clk_74 =0;
   delay_ms (5);
   clk_74 =1;//ÉÏÉıÑØÓĞĞ§
   delay_ms(100);
   Init_LCD16032_DS3234();//ÏÔÊ¾ÆÁ³õÊ¼»¯
   ad7730_init();//AD7730³õÊ¼»¯
   DS3234_READSRAM_VALUE(); //¶Á³ö´æÈë´æÈëSRAMµÄ¸÷ÖÖ²ÎÊıreadpra(); //´æ·ÅÄ¬ÈÏ±äÁ¿²ÎÊı  DS3234
   check_pra();//´æ´¢²ÎÊıÊÇ·ñÕıÈ·  ²»ÕıÈ·°ÑÄ¬ÈÏ²ÎÊıĞ´ÈëEEPROM //ÏÖÔÚ Èç¹ûÏÂÔØÄ¬ÈÏ²ÎÊı£¬½«µ¼ÖÂDS3234±äÁ¿±»³õÊ¼»¯ 
   CLI();
   readpra();   //ÔÚ¶Áµ½eeprom
   readpra1();	
   CLI();	
   display_check();//ÏÔÊ¾¼ì²é Èç¹ûÊäÈëkey_input¼ü£¬ÊäÈëÏàÓ¦µÄÊı¾İ»á×°ÈëÄ¬ÈÏ²ÎÊı  ÊäÈëËø»ú¿ªÊ¼Ê±¼ä
   CLI();
   calc_dat();	//ÏµÊı¼ÆËã
   if(z1<0){z1=0;z1_temp=0;}//z1×ÜÀÛ¼ÆÁ¿ z1_temp 100msÀÛ¼ÆÁ¿
   if(totalshift[0]<=0)totalshift[0]=0;//Z1//°à×éÀÛ¼Æ
   if(totalshift[1]<=0)totalshift[1]=0;//Z2
   if(totalshift[2]<=0)totalshift[2]=0;//Z3ÀÛ¼ÆÁ¿ ´æ´¢ÓÚDS3234ÖĞ£¨12C887£©
   run=running=0;//ÔËĞĞÎ» 
   for(i=0;i<press_ak;i++)averpress[i]=0;//press_ak=20 //Ñ¹Á¦Æ½¾ù»º´æµ¥Ôª
   press=press_1=0;
   sumquality=addat*presspra-tarevalue;//ÖØÁ¿ kg  //Ã»ÓĞÓÃ ×ÜÖĞ¶Ï±»¹Ø
   dissignal();//²Ëµ¥»­ÃæÏÔÊ¾±£³Ö½çÃæº¯Êı
   d_out=1;//ºó¼Ó
   if(!keymode)//·Ç¼üÅÌÄ£Ê½  keymode =1 ¼üÅÌÄ£Ê½  keymode =0 Í£¼üÅÌÄ£Ê½ ±¸Í×Êä³öÔÚ¶¨Ê±Æ÷ÀïÒ²ÓĞ£¬
   {
	    dout(7,0);//³ÓÍ£Ö¹
		  dout(8,1);//±¸Í×ĞÅºÅÊä³ö  ³ÌĞòÒÑ¾­¿ªÊ¼Ö´ĞĞ£¬Ö¤Ã÷
      if((xram[43]==1)&&(xram[46]==1))//P43´®ĞĞÍ¨Ñ¶µØÖ·Ñ¡Ôñ1ÓĞ´®ĞĞÍ¨Ñ¶£¬0ÎŞ´®ĞĞÍ¨Ñ¶¡£//Æô¶¯Éè¶¨ÖµµÄÀ´Ô´¶¼À´×Ô´®¿Ú 
		  status[0]=0x31;//ÓĞ±¸Í×  //´®¿Ú·¢ËÍÓÃ
 		  else //Æô¶¯Îª¿ª¹ØÁ¿x3:3,4  Éè¶¨ÖµÓÉÄ£ÄâÁ¿x6:1,2ÊäÈë»òÊÇÓÉ¼üÅÌ¸ø¶¨
		  status[0]=0x30;//ÎŞ±¸Í×
   }
    else//¼üÅÌÄ£Ê½  Æô¶¯ºÍÉè¶¨Öµ¾ùÓĞ¼üÅÌÉè¶¨
   {
	    dout(8,0); //¼üÅÌÄ£Ê½ ²»ĞèÒªÊä³ö±¸Í×
		  status[0]=0x30; //´®¿Ú·¢ËÍÓÃ £¨ÎŞ±¸Í×£©
   }
   delay_ms(1);
////d_out=1;//¿ª¹ØÁ¿ÔÊĞíÊä³ö //ÓĞÎÊÌâ ´ı¸Ä
   SEI();

 while(1)
{ 
  delay_us(3);
	check_day();//¼ì²éËø»úÊ±¼ä //Ëø»úÊ±¼äµ½day_locking_falg=1 check_day_flag=1; xram[125]=1;ÏÂÔØÄ¬ÈÏ²ÎÊı ¶Á²ÎÊı   ÓĞĞ´flash
	delay_us(3);
	recive_key();
	delay_us(3);
	if(((jgdis>1)||keyflag)){dissignal();jgdis=0;}//¼üÅÌ°´ÏÂÃ»ËÉ¿ª¼üÅÌ ±êÖ¾Î»µÈÓÚ1  ±íÃ÷°´ÏÂ°´¼üÏÔÊ¾
	else jgdis++;

	/*****************************Á÷Á¿±¨¾¯´¦Àí***********************************///(Ö¸Ê¾µÆ±¨¾¯ Êı×ÖÁ¿Êä³ö)
//     if(din(0))clear_warn();//¿ª¹ØÁ¿ÊäÈë  ¹ÊÕÏÓ¦´ğ DI1 //´ËÇå³ı¾¯¸æº¯ÊıÎŞĞ§   Ã»ÓÃ Ã»ÓÃ
	if(maxwarn){dout(1,1);dout(11,1);} //×î´óÖµÊı×ÖÁ¿Êä³ö±¨¾¯£¬×î´óÖµÖ¸Ê¾µÆ±¨¾¯  DO2
	else {dout(1,0);dout(11,0);}		   
	if(minwarn){dout(3,1);dout(12,1);}//×îĞ¡ÖµÊı×ÖÁ¿Êä³ö±¨¾¯£¬×îĞ¡ÖµÖ¸Ê¾µÆ±¨¾¯   DO3           
	else {dout(3,0);dout(12,0);}//ÆäËûÇé¿ö£¬×îĞ¡ÖµÊä³ö±¨¾¯¹Ø£¬×îĞ¡ÖµÖ¸Ê¾µÆ±¨¾¯¹Ø

	/********************************ÊÂ¼şĞÅÏ¢¼ì²â*************************************/
    ////////ÅÜÆ«¿ª¹Ø´¦Àí/////// ÅÜÆ«ºóÒÇ±í³ÓµÄÍ£Ö¹  Æ¤´ø³ÓÓÃ£¬×ª×Ó³Ó¿ÉÄÜ²»ĞèÒª
	if(din(2)) h9=0;//DI3=0 ¿É½ÓÅÜÆ«¿ª¹Ø//Ã»ÓĞÅÜÆ«//ÊÍ·Å    //Æ¤´øÅÜÆ«»òÏÖ³¡¿ØÖÆ       ÊäÈëÎª0   £¨¿ª¹Ø±ÕºÏÊ±£© //DI3
	else//DI3=1//ÓĞÅÜÆ«  £¨¿ª¹Ø¶Ï¿ªÊ±£©                                         ÅÜÆ«ÏÖ³¡È·¶¨  
	{
		h9=1;
		if(xram[41]==1)//1 ÅÜÆ«¿ª¹Ø¶Ï¿ªÊ±£¬ÒÇ±í¿ØÖÆÆ÷µçÁ÷»·Í£Ö¹//0 ÅÜÆ«¿ª¹Ø¶Ï¿ªÊ±£¬ÒÇ±íµçÁ÷»·²»Í£Ö¹
		{
// 			r_temp[9]=0x30;//Ã»ÓĞÓÃµ½´ËÊı×é
			stop();//run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0; 
		}
	}
	////////×ª×Ó³Ó¸ºÔØÊÂ¼ş ////////  P10ºÉÖØ´«¸ĞÆ÷¸ºÔØµÄ×ÜÈİÁ¿  Ä¬ÈÏÖµ60¹«½ï
	if((sumquality+tarevalue)>(xram[10]*1.1)){h4=1;l4=0;}//ºÉÖØ´«¸ĞÆ÷¸ºÔØ*1.1/ÓĞĞ§³ÆÁ¿¶Î³¤¶È//H4 ³ÆÖØÏµÍ³¹ıÔØ//³¬¹ı³ÆÖØ´«¸ĞÆ÷ºÉÖØ¸ºÔØµÄ1.1±¶  66¹«½ï
	else h4=0;
	
	
	if((sumquality+tarevalue)<(xram[10]*0.03))//ºÉÖØ´«¸ĞÆ÷¸ºÔØ  1.8¹«½ï
	{
		
			l4=1;//³ÆÖØÏµÍ³¸ºÔØ¹ıĞ¡
			h4=0;
		
	}
	else l4=0;
	 ////////×ª×Ó³ÓÁ÷Á¿ÊÂ¼ş //////// kg/h
	if((tracevalue>xram[21])&&(running>5))//Á÷Á¿ÉÏÏŞ ¼ÆËã È·¶¨ÒÇ±íÈ·ÊµÊÇÆô¶¯ÁË//P21Á÷Á¿ÉÏÏŞ Æô¶¯Ê±¼ä´óÓÚ5S   Æô¶¯Ê±¼ä´ı¸Ä
	{
		if(tracevalue>xram[21])//Á÷Á¿³¬¹ıÉÏÏŞ   
		{
			h1=1;l1=0;//Êµ¼ÊÁ÷Á¿³¬¹ıÉÏÏŞÖµ
			maxwarn=1;//¿ª¹ØÁ¿ÉÏÏŞÊä³ö
			minwarn=0;
		}
	}
	else
	{
		maxwarn=0;
		h1=0;
	}
	if((tracevalue<=xram[22])&&(running>5))//Á÷Á¿ÏÂÏŞ tracevalueÊÇÊµ¼ÊÆ½¾ùÁ÷Á¿Öµ//P22Á÷Á¿ÏÂÏŞ Æô¶¯Ê±¼ä´óÓÚ5S
	{
		if(tracevalue<=xram[22])//Á÷Á¿ÏÂÓÚµÈÓÚÏÂÏŞ
		{
			l1=1;h1=0;//Êµ¼ÊÁ÷Á¿µÍÓÚÏÂÏŞÖµ
			minwarn=1;//Á÷Á¿ÏÂÏŞ±¨¾¯
			maxwarn=0;
		}
	}
	else
	{
		minwarn=0;
		l1=0;
	}
	
	////////×ª×Ó³ÓºÉÖØÊÂ¼ş ////////  kg
	if((whtload>xram[23])&&(running>5))//ºÉÖØÉÏÏŞ //ÂË²¨ºóµÄºÉÖØÆ½¾ùÖµ£¨20£© //P23ºÉÖØÉÏÏŞ  Æô¶¯Ê±¼ä´óÓÚ5S   ÉÏÏŞ200
	{
		if(whtload>xram[23])//¸ººÉ³¬³öÉÏÏŞ 
		{
			h2=1;l2=0;
		}
	}
	else h2=0;
	if((whtload<xram[24])&&(running>5))//ºÉÖØÏÂÏŞ //ÂË²¨ºóµÄºÉÖØÆ½¾ùÖµ£¨20£© //P24ºÉÖØÏÂÏŞ  Æô¶¯Ê±¼ä´óÓÚ5S   ÏÂÏŞ5
	{
		if(whtload<xram[24])
		{
			l2=1;//¸ººÉµÍÓÚÏÂÏŞ 
			h2=0;
		}
	}
	else    l2=0;
	////////Á÷Á¿Æ«²îÊÂ¼ş¼ì²â¼°´¦Àí////////
	if((fabs(setvalue-tracevalue)>xram[37])&&(running>5)){h5=1;dout(4,1);}//Á÷Á¿Æ«²î·¶Î§³¬³ö //P37Á÷Á¿Æ«²î·¶Î§ Æ«²îÊä³ö  Éè¶¨Öµ¼õ·´À¡
	else {h5=0;dout(4,0);}//Æ«²îÊä³ö¹Ø
	////////Á÷Á¿Éè¶¨ÖµÊÂ¼ş¼ì²â////////
	if((!avmode)&&(setvalue<(xram[1]*0.02)))e5=1;//ÔÚ·ÇÈİ»ıÄ£Ê½£º//Éè¶¨ÖµµÍÓÚµÍÏŞ  P1¶î¶¨¸øÁÏÁ¿
	else e5=0;
	////////´«¸ĞÆ÷ÊäÈë´íÎóÊÂ¼ş¼ì²â//////// £¨´ı¸Ä£©
	if((addat<30||(addat>65532))&&(running>5))c1=1;//´«¸ĞÆ÷ÊäÈë³ö´í  //24Î» ÓÃÁË16Î» 
	else  c1=0;//Õı³£
	////////µç»ú¹ÊÕÏ////////if((speedcode<3)&&xram[9])speedless++;else speedless=0;//ËÙ¶È´«¸ĞÆ÷ÎŞĞ§//ËÙ¶È´«¸ĞÆ÷ÓĞĞ§  P9ËÙ¶È´«¸ĞÆ÷ÊÇ·ñÎªÓĞĞ§¡£ 0ËÙ¶È´«¸ĞÆ÷ÎŞĞ§£¬ËÙ¶ÈÓÉP5¾ö¶¨¡£1ËÙ¶ÈÓÉ´«¸ĞÆ÷ÊäÈë	            	
	if((speedless>10)&&run&&(ctrlout>2000))  h7=1;//Á¬Ğø10s²»Õı³£Ê±//speedcodeĞ¡ÓÚ3 ³¬¶à10s  //ËÙ¶È¸ø¶¨2000
	else  h7=0;	                               //´ı¸Ä
	////////ËÙ¶È´«¸ĞÆ÷ÊäÈë´íÎó////////
	if((speedcode>1500)&&xram[9]&&(running>5))  //´ı¸Ä
	{
		if((speedcode>1500)&&xram[9])c3=1;//75*20 ËÙ¶È´«¸ĞÆ÷ÊäÈë´íÎó
		else c3=0;
	}	
	////////ËÙ¶ÈÊä³ö¿ØÖÆÊÂ¼ş///////
	if(((ctrlout/speedcal)>=xram[31])&&run)h6=1;//¿ØÖÆÊä³ö×î´óÖµ //¿ØÖÆÆ÷µ½ÉÏÏŞ  ¸ù¾İ¹«Ê½£º20.8845*B/4096 =Êä³öµçÁ÷  ´ı¸Ä
	else h6=0;
	////////Á÷Á¿Éè¶¨ÖµÊäÈë´íÎó///////
	if(((setvaluetemp>xram[1]*3)&&avmode)||(!avmode&&(setvaluetemp>xram[1])))s9=1;//Éè¶¨Öµ³¬³ö¼«ÏŞ//Á÷Á¿ÉÏÏŞ
	else s9=0;
	////////ËÙ¶È´«¸ĞÆ÷¼ì²â///////
// 	if((speed_check==0)&&xram[9])c2=1;//¶ÁÈ¡ËÙ¶È¼ì²â¶Ë¿Ú£¬µÈÓÚ0 ËÙ¶È±¨¾¯   //ËÙ¶È´«¸ĞÆ÷ÊäÈë³ö´í
// 	else c2=0;
	////////Ô¤¸øÁÏ»úµÄÆô¶¯Í£Ö¹///////  ´ı¸Ä ÏÖ³¡½»Á÷
	if(pre_feeder&&(run))dout(5,1);//Ô¤¸øÁÏ»úÆô¶¯   Ô¤¸øÁÏ±êÖ¾1    
	else dout(5,0);//Ô¤¸øÁÏ»ú¹Ø±Õ	
	
/**********************¼ì²â±äÁ¿*******************/	
	if((dis_signal<1)||(dis_signal>10))dis_signal=2;//ÏÂĞĞÏÔÊ¾±äÁ¿
	if((pre_feeder<0)||(pre_feeder>1))pre_feeder=0;
	if((keymode<0)||(keymode>1))keymode=0;//¼üÅÌÄ£Ê½±ê¼ÇÎ»
	if((avmode<0)||(avmode>1))avmode=0;
	if(jgdis<0)jgdis=0;    
 }		
}	  
/**********************ËÙ¶È¼ÆÊıÖĞ¶Ï*******************/	//×ª×ÓËÙ¶È¿ØÖÆ   //Ö»ÓĞÔËĞĞÊ±£¬³ÆÖØĞÅºÅ²ÅÄÜ¸üĞÂ
void TIM8_UP_IRQHandler  (void)   //TIM8ÖĞ¶Ï  ×ª×ÓËÙ¶ÈµÄ¿ØÖÆÖÜÆÚ
{
 if(TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)  //¼ì²éTIM8¸üĞÂÖĞ¶Ï·¢ÉúÓë·ñ
 {  
	   u16 i,j,avt;//
	   u32 addata=0,count,a=0;
	   TIM_ClearITPendingBit(TIM8, TIM_IT_Update);  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾ //×¢Òâ´Ë³ÌĞò  ²»Çå³ı²»ÄÜ½øÈëÖĞ¶Ï   ¶¨Ê±Æ÷¼ÆÊıÆ÷Ò»Ö±ÔÚ¼ÆÊı
// 	 TIM_SetCounter(TIM8, 0);//¼ÆÊıÆ÷ÇåÁã
	 /////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ±¸·İ//////// 
	   s_val=SysTick->VAL;//²»Îª0 Ôò 16Î» Îª0 Îª0 16Î»ÇåÁã
	   if(s_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//¹Ø±Õ¼ÆÊıÆ÷1
	   s_ctrl=SysTick->CTRL;
// 	 cpu_led=~cpu_led;
	   plusmul++;  //ºÍËÙ¶È¶¨Ê±Æ÷
	   avt=tare_s; //Æ¤ÖØµÄ¶ÔÓ¦Öµ
     tare_s++;
  /**************************** AD7730ÂëÖµ´¦Àí*********************************///ÓÃÁË16Î»¾«¶È
//if(Zero_flag==1)
//{
    ad_flag=1;//¿ØÖÆADÖĞ¶ÏµÄ¶ÁÈ¡
		for(i=0;i<26;i++)
		{
		  ad7730_code_result[i]=ad7730_filter[i];  //Êı¾İ×ªÒÆ
		}
		qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//¶ÔÊı¾İÅÅĞò  ´ÓĞ¡µ½´óÅÅĞò
		addata=0;
		for(i=5;i<21;i++)
		{
			addata+=ad7730_code_result[i];//5µ½20Ïà¼Ó  16¸öÊı 
		}
		addata>>=12;//×óÒÆ12Î»£¬³ıÒÔ4096,£¿×óÒÆ4Î»£¬³ıÒÔ16£¬×óÒÆ8Î»³ıÒÔ256//È¥ÁË8Î»¾«¶È //³ıÒÔ16£¬ÇóÈ¡ÂëÖµµÄÆ½¾ùÖµ  24Î»È¥ÁË8Î»¾«¶È 16Î»
		addat=addata;//16Î»ÂëÖµ   //100ms¸üĞÂÒ»´Î
		ad_flag=0; //¿ØÖÆADÖĞ¶ÏµÄ¶ÁÈ¡
	  sumquality=addat*presspra-tare_date_result[avt];
		
//}	

	 //Æ¤´ø¸ººÉ  ¼õÈ¥Æ¤ÖØ    //ĞèÒªĞŞ¸Ä  kg £¨ÏÖ³¡ÊÇ·ñĞèÒªÂË²¨´¦Àí£©	 ÂëÖµ£¨ÓÃÖÊÁ¿´ú±íÁ¦£©  ´ú±í×ÅÁ¦   Æ¤ÖØ´ú±í×ÅÁ¦
	 /******************************ÔËĞĞÄ£Ê½×ª±ä**************************/ 
	  plusecount++;//µ¥Î»Ğı×ªÂö³å¼ÆÊı
// if(tare_flag1==1&&tareline==1)    //³ıÆ¤±ê¶¨£¬±ê¶¨È¦Âö³åÊı //½øÈëÖĞ¶ÏºóÅĞ¶ÏÊÇ·ñ´ÓÁãµã¿ªÊ¼³ÆÖØ
// 		{ 
// 			j=tarecount;  //Æ¤ÖØ´ÎÊı
//      	a=addat;      
// 			tare_date[j]+=a;			 //½«ÂëÖµ¼Óµ½Êı×éÀïÃæ¶ÔÓ¦µÄµÚjÎ»
// 	    tarecount++;           //Êı×é´ÎÊı+1
//    }
	  if(modecount==1) //ÇåÁÏ  µ²°å¹Ø ×ª×Ó°´Éè¶¨×ªËÙĞı×ª
    { 
		 EXTI_Set_Int(1);
     plusesum+=plusecount*xram[63];//ÔËĞĞÄ£Ê½1£¬¼ÇÂ¼Ğı×ªÂö³å£¬°Ñ»ıÀÛµÄÁÏ×ª³öÈ¥  xram[63]·Ö¸ôÂö³åÊı
		 if((plusesum>=2*xram[64])&&Zero_flag==1) //ÀÛ¼Æ½Ç¶È´óÓÚÓĞĞ§½Ç¶È//Ğı×ª×ÜÂö³åÊı´óÓÚÓĞĞ§½Ç¶ÈÂö³åÊı  
		 {
			plusesum=0;  //Ğı×ªÂö³å¼ÆÊı
			modecount=2;	//Ä£Ê½¼ÆÊı
		  plusecount=0;//¼ÆÊıÖĞ¶ÏµÄ¼ÆÁ¿
			EXTI_Set_Int(0);
     }
    }
	   else 
		 if(modecount==2) //Ä£Ê½2Ê±¼äµ½×ª»»µ½Ä£Ê½3  //µ²°å¿ªÏÂÁÏ    
		 {
			 plusesum+=plusecount*Sepapulse;//¼ÇÂ¼×ÜÀÛ¼ÆÂö³å
			 if(plusesum>=2*xram[64]) //Ğı×ª×ÜÂö³åÊı´óÓÚÓĞĞ§½Ç¶ÈÂö³åÊı
			 {
				  plusesum=0;//ÀÛ¼ÆÂö³åÊıÇåÁã
				  modecount=3;//Ä£Ê½3
			    plusecount=0;//¼ÆÊıÖĞ¶ÏµÄ¼ÆÁ¿
			 }
		 }
	   /**********¿ØÖÆ²ÎÊıÇóÈ¡£¨q1ÓÃÓÚÇóÈ¡Á÷ÈëÁ÷Á¿ºÍµ²°å¿ØÖÆ//qnÇóÈ¡·´À¡Á÷Á¿ºÍ¿ØÖÆËÙ¶È£©*************/
		if(modecount==2||modecount==3)
		 {    
			   
				 partmoment=0;
				 for(count=separa;count>1;count--) //1µ½seprara 
				 {  
					 Q_quality[count] =Q_quality[count-1];//Êı×éÓÒÒÆ	  ·Ö¸ôĞı×ª
				 } 				 
				 for(count=2;count<=separa;count++)//¼ÆËãÁ¦¾Ø
				 {
					 partmoment+=Q_quality[count]*Q_lenth[count];  //³¤¶Èµ¥Î»CM
				 }
				 Q_quality[1]=(sumquality*xram[70]-partmoment)/Q_lenth[1];//Çó³öq1  ÓÃ×÷µ²°å¿ØÖÆ  P70: L  ¶¯Á¦±Û*¶¯Á¦=×èÁ¦±Û*×èÁ¦±Û  ÓĞ¿ÉÄÜ³öÏÖ¸ºÊı¡£ÓÉÓÚ¼ÆÁ¿µÄ²»×¼È·ĞÔ
				 whtload =0;
				 for(count=1;count<=separa;count++) //Ö»ÓĞÔËĞĞµÄÊ±ºò ²ÅÓĞ¸ººÉ  ²»ÔËĞĞÊ±£¬¸ÄÏÔÊ¾ÄÇ¸ö¸ººÉ  
				 {
              whtload+=Q_quality[count]; //¼ÆËã×Ü¸ººÉ
         }
				 if(modecount==3) //Õı³£Ä£Ê½3   ÓÃÒÔÏÂ¹«Ê½
				 {	 
				   if(Q_quality[separa]>0)
				  {
				   speed_set=(setvalue*angledive)/(Q_quality[separa]*3.6);	//ÇóµÃ¸ø¶¨ËÙ¶È  setvalue=Q_quality[separa]/angledive*speed_set*3.6 =kg/¡ã*¡ã/s*3.6
				  } 
				 }  
			}
		 
	     /*******************************Á÷Á¿µÄ¼ÆËã´¦Àí*******************************************/  //Á÷Á¿¾ùÖµµÄÇóÈ¡
	  if(modecount==1)  //¸ÕÆô¶¯Ê±£¬¿ØÖÆÄ£Ê½1  ÇåÁÏµÄ¹ı³Ì
		  {
// 			 press=sumquality/(xram[64]-anglemode);//Ä£Ê½1µÄÁ÷Á¿¿ØÖÆ   Ä£Ê½1Ê£Óà½Ç¶È
			   press=0;//¼ÆËãÁ÷Á¿ÓÃ
			   Xn=0;//¼ÆËãÁ÷Á¿ÓÃ
		  }
		else if((modecount==2)||(modecount==3)) //2 3
		  {
			 	 press=Q_quality[separa]/angledive;//µ¥Î»¶ÈÊıµÄÖØÁ¿  kg/¶È   //Á÷Á¿·´À¡ÓÃ    //Ã¿¸öÉÈĞÎµÄ½Ç¶Èangledive
			   Xn=Q_quality[1]/angledive;// kg/¶È   µ²°å¿ØÖÆÓÃ   (·Ö¸ô½Ç¶È)  //£¨63 ·Ö¸ô½Ç¶ÈÂö³åÊı£©/£¨77 Ò»È¦µÄ×ÜÂö³åÊı£©*360
		  }
			press=press_1+aq*(press-press_1);//AQºÉÖØÂË²¨  //ÂË²¨ºóµÄºÉÖØ  //
			press_1=press;
			for(i=1;i<press_ak;i++)averpress[i-1]=averpress[i];//press_ak=20  Êı×éÏò×óÒÆ¶¯ 0-press_ak-1   20¸ö µ½19
			averpress[press_ak-1]=press;
			pressvalue=0;  //
			for(i=0;i<press_ak;i++)pressvalue+=averpress[i];
			pressvalue/=press_ak;//ÂË²¨ºóµÄºÉÖØÈ¡¾ùÖµ   20¸ö¾ùÖµ  //ÏÔÊ¾Á÷Á¿µÄ¼ÆËãÓÃµ½´Ë±äÁ¿  
			
		/************************************ËÙ¶ÈµÄ¿ØÖÆ***************************************/ 
  if((run)&&(!calflg))//ÔËĞĞ²¢ÇÒ·Ç±ê¶¨Ä£Ê½  
	{
			////////////////////Çı¶¯ºÍÓ¦´ğ////////////////////
		 
			if(modecount==1)//¸ÕÆô¶¯¿ØÖÆÄ£Ê½1 ¹Øµ²°å ¿ª×ª×ÓËÙ¶È £¨Ê£ÓàµÄÁÏÂ©ÏÂÈ¥£©
			{
			    ctrlout=xram[39]*speedcal;//P39Èİ»ı·½Ê½Çı¶¯µçÁ÷ 4096/22.9  ×î´óÂëÖµ¶ÔÓ¦µÄ×î´óµçÁ÷ //´ı¸Ä  £¨ÒÔÈİ»ı·½Ê½¿ØÖÆ×ªËÙ£
      }
			else if(modecount==2)  //¿ªµ²°åºÍ×ª×ÓËÙ¶È  ·Ö¸ôµ¥Ôª¿ªÊ¼¼Ç·Ö¸ôµ¥ÔªµÄÖØÁ¿
			{   
          ctrlout=xram[39]*speedcal;//P39Èİ»ı·½Ê½Çı¶¯µçÁ÷ 4096/20.88  ×î´óÂëÖµ¶ÔÓ¦µÄ×î´óµçÁ÷ //´ı¸Ä  £¨ÒÔÈİ»ı·½Ê½¿ØÖÆ×ªËÙ£©         				
      }
      else if(modecount==3)  //Èİ»ı·½Ê½£¨ÊÖ¶¯Ä£Ê½£©µ²°åµÄ¿ØÖÆ  ËÙ¶ÈµÄ¿ØÖÆ·ÅÔÚ¼ÆÊıÆ÷Àï
			{   
				if(avmode||(vmode))//Èİ»ı·½Ê½¡£ÔÚÖØÁ¿¹¤×÷µ±ÖĞ£¬Æô¶¯¿ªÊ¼Ê±£¬ÏÈ°´ÒÔÏÂ¿ØÖÆÒ»¶ÎÊ±¼ä
				{  
				  ctrlout=xram[39]*speedcal;//P39Èİ»ı·½Ê½Çı¶¯µçÁ÷ 4096/22.9  ×î´óÂëÖµ¶ÔÓ¦µÄ×î´óµçÁ÷ //´ı¸Ä   
					ctrlout+=avdanum;
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //Èİ»ı·½Ê½×ª×Ó×ªËÙµÄ¿ØÖÆ
			  }
// 		  else if((xram[38]==1)&&(h5==0)){}//Æ«²î·¶Î§ÄÚ²»µ÷ËÙP38Æ«²î¿ØÖÆÑ¡Ôñ£ºµ±Ñ¡Ôñ0Ê±£¬ÔÚÈÎºÎÊ±¼äÄÚµ÷½Úµã»÷ËÙ¶È¡£µ±Ñ¡Ôñ1Ê±£¬ÔÚÆ«²î·¶Î§ÄÚ²»µ÷½Úµç»úËÙ¶È  ÔÚÆ«²î²»³¬ÏŞÊ±
				else//ÖØÁ¿¹¤×÷·½Ê½                               //Ñ¡Ôñ´Ë²ÎÊı¿É¼õĞ¡µç»úÕğµ´£¬Ê¹µÃÎïÁÏÔËĞĞÆ½ÎÈ¡£ 
				{
				  if((bflag==0)||(((zb-zi)>0)&&bflag))//ÅúÁ¿Éú²ú±êÖ¾Î»0 bsetend 5*set_value/3600//Á÷Á¿Éè¶¨Öµ  ((zb-zi)>bsetend)&&bflag)  zbÅúÁ¿Éú²úÉè¶¨Öµ ziÅúÁ¿Éú²úÔöÁ¿
				  {   
					ctrlout=speed_set/xram[69]*analogpra1+analog1; //xram[68]=0//ËÙ¶È¶ÔÓ¦ÂëÖµ/ËÙ¶È=µ¥Î»ËÙ¶ÈÂëÖµ
					if(ctrlout<=damin) ctrlout=damin;
					if(ctrlout>=damax) ctrlout=damax;
					if(ctrlout>4095)   ctrlout=4095;   //Èİ»ı·½Ê½×ª×Ó×ªËÙµÄ¿ØÖÆ
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //ÅúÁ¿Éú²ú½áÎ²¿ØÖÆ  È¥µô
					{
						if(bctr==0)
						{
							danum=ctrlout/220;
							bctr=1;
						}
						ctrlout-=danum;//100msµÄ¿ØÖÆÖÜÆÚ  »ºÂıµÄ¼õĞ¡
						if(ctrlout<(damin+300)) ctrlout=damin+300;
						else if(ctrlout>damax) ctrlout=damax;
						if(ctrlout>4095) ctrlout=4095;
					}
				}
		 }
  }
		if(mainboardflag==0)//µ±Ö÷°åÁ÷Á¿Êä³öĞ£×¼Ê±£¬´Ë±êÖ¾Î»ÖÃ1 ²»¿ÉÒÔÊä³ö
		{
		   DAC7612(0,ctrlout);
		}//ËÙ¶È¿ØÖÆÊä³ö       ÕâÀï»á¼ÓÉÏÒ»¸öµ²°å
 /////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ»¹Ô­////////
			SysTick->LOAD=s_val;
			SysTick->VAL=0x00;
			SysTick->CTRL=s_ctrl;
	 
  }

}

/*********************************¶¨Ê±Æ÷3±È½ÏÖĞ¶Ï********************************/

//¶¨Ê±Æ÷3ÖĞ¶Ï·şÎñ³ÌĞò
void TIM3_IRQHandler (void)//TIM3ÖĞ¶Ï
{
 if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //¼ì²éTIM3¸üĞÂÖĞ¶Ï·¢ÉúÓë·ñ
 {            
	  float speed_value=0;
	  uint16_t i,DS3234_code;
	  uint32_t ls_sec,addatab;
	  TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾ //×¢Òâ´Ë³ÌĞò  ²»Çå³ı²»ÄÜ½øÈëÖĞ¶Ï   ¶¨Ê±Æ÷¼ÆÊıÆ÷Ò»Ö±ÔÚ¼ÆÊı
	  /////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ±¸·İ//////// 
		t_val=SysTick->VAL;//²»Îª0 Ôò 16Î» Îª0 Îª0 16Î»ÇåÁã
		if(t_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//¹Ø±Õ¼ÆÊıÆ÷1
		t_ctrl=SysTick->CTRL;
	   ///////////////////////////////////////////
		t_tmp++;//*
		flash_light=!flash_light; //È¡·´Ö¸Áî//¹ÊÕÏ±¨¾¯Ö¸Ê¾µÆÓÃ
	   ///////////////////////////////////////////
	  if(run==0&&flashblag==1)  //¼ÆËã²ÎÊıÔÚ¸Ä±ä²ÎÊıÊ±¼Ó ÔÚ³ÓÔËĞĞÊ± ĞŞ¸Ä²ÎÊıÊ±Ö´ĞĞ
	  {
        writepra();
	     flashblag=0;  
    }
    /////////////////////////////////ËÙ¶È¼ÆËã///////////////////////////////////	
		if(xram[9])//ËÙ¶È´«¸ĞÆ÷ÊÇ·ñÓĞĞ§£¿1£¬ÓĞĞ§£¬·ñ²àÎŞĞ§¡£ÎŞĞ§Ê±£¬ÓÉ¶î¶¨ËÙ¶ÈÈ·¶¨
		{
			speedcode=plusmul*Sepapulse+TIM_GetCounter(TIM8)-pluscountup;//¼ÆÊıÆ÷Âö³åÊı  P77:·Ö¸ôÂö³åÊı
			pluscountup=TIM_GetCounter(TIM8);//¼ÇÂ¼±¾´Î¶¨Ê±µÄÂö³åÖµ
			plusmul=0;//±¶ÊıÇåÁã
			speed_count++;//¶¨Ê±Æ÷ÖĞ¶Ï´ÎÊı
			speedcode_tal+=speedcode;//¼ÆÊıÏà¼Ó //×ª×Ó³Óspeedpra£¨µ¥¸öÂö³å×ª×ÓĞı×ªµÄ½Ç¶È£©
// 			anglecount =speedpra*speedcode;//¼ÇÂ¼Ğı×ª½Ç¶È 100ms×ª×Ó³Ó×ªÁË¶àÉÙ½Ç¶È
			speed_value=10*speedpra*speedcode;//¼ÆËãËÙ¶È¡£speedpraµ¥¸öÂö³å¶ÔÓ¦Æ¤´øËù×ß³¤¶È  ÎªÊ²Ã´³ËÒÔ10   Ã¿100msÆ¤´ø×ßÁËx ,1s×ßÁË10*x =speed_value 1SÆ¤´øĞĞ×ßµÄ³¤¶È
			speedvalue=0.6*speedvalue+0.4*speed_value;//¼ÓÈ¨ speedvalueÁ÷Á¿¼ÆËãÊ±ÓÃ 
			if(speed_count==10)//10¸öÊıÆ½¾ù  1s
			{
				speed_code=speedcode_tal/10.0;
				speed_v=10*speedpra*speed_code;//²ÉÑù10´ÎµÄÆ½¾ùËÙ¶È  M/s
				speed_dis=speed_v1+av*(speed_v-speed_v1);//Æ½¾ùËÙ¶ÈÂË²¨
				if(speed_dis<0)speed_dis=0; 
				speed_v1=speed_dis;//ËÙ¶ÈÖµ //²»ÇåÁã  put_disdatº¯Êıµ÷ÓÃ  //ÔÚdissignal(void)µ÷ÓÃ Ö÷ÏÔÊ¾½çÃæ speed_v1Ã»ÓĞÓÃ
				speed_hz=speedcode_tal;//×ÜÂö³åÊı  ¶¨Ê±Æ÷ÖĞ¶Ï10´ÎµÄ×ÜÂö³åÊıput_disdatº¯Êıµ÷ÓÃ  ËÙ¶ÈÆµÂÊ  1s¼ÆÊıÆ÷¼ÆÊıÖµ 
				speedcode_tal=0;
				speed_count=0;
			}
		}
		else{speedvalue=xram[5];}//*/ //¶î¶¨ËÙ¶È  m/s  0.1  ·´À¡Á¿ÓÃ¶î¶¨ËÙ¶È
		
	
// 		if(Timedata>=2) //300ms³õÊ¼»¯AD7730  //ÈçºÎÀí½âÕâ¸ö³õÊ¼»¯     //´ı¸Ä
// 		{
// 			c4=1;//C4AD7730²ÉÑù´íÎó
// 			EXTI->IMR &= ~(EXTI_Line15);// ÆÁ±ÎÍâ²¿ÖĞ¶Ï15;//¹Ø±ÕÍâ²¿ÖĞ¶Ï
// 			USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//´®¿Ú3½ÓÊÕÊ¹ÄÜ½ûÖ¹
// 			
// 			AD_RST =0;
// 			delay_us(40);
// 			AD_RST =1;//1
// 			delay_us(40);
// 			Timedata=0;
// 			ad7730_init();
// 			EXTI->IMR |= EXTI_Line15;//Ê¹ÄÜÍâ²¿ÖĞ¶Ï15;//¿ªÍâ²¿ÖĞ¶Ï AD7730ÖĞ¶Ï
// 			USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//´®¿Ú3½ÓÊÕÊ¹ÄÜ½ûÖ¹//´®¿Ú3½ÓÊÕÖĞ¶ÏÊ¹ÄÜ¿ª
// 		}
		Timedata++;//*/ AD7730³õÊ¼»¯ÓÃ  Ã¿200ms³õÊ¼»¯Ò»´Î
		DS3234_WRITESRAM_VALUE();//²¿·Ö¶¨Òå±äÁ¿ÏòsramÀïĞ´
		/*****************************¶ÁÈ¡µ±Ç°Ê±¼ä*********************************/
		DS3234time[0].second=Read_data_DS3234(0x00); //second  0  
		DS3234time[0].minute=Read_data_DS3234(0x01); //minute
		DS3234time[0].hour=Read_data_DS3234(0x02);   //hour
		DS3234time[0].week=Read_data_DS3234(0x03);   //week
		DS3234time[0].date=Read_data_DS3234(0x04);   //date
		DS3234time[0].month=Read_data_DS3234(0x05);  //month
		DS3234time[0].year=Read_data_DS3234(0x06);   //year//*/
		/*****************************¶ÁÎÂ¶ÈÂëÖµ*********************************/
		if(temper_flag==1)  //±êÖ¾Î»ÔÚ·şÎñº¯ÊıÀïµ÷ÓÃ  Ö»ÓĞÎª1²Å¶ÁÈ¡ÎÂ¶È  ÔÚ·şÎñÖµÀïÓÃµ½
		{
			DS3234_status_flag=Read_data_DS3234(0x0f);//BSY bit status
			if((DS3234_status_flag&0x04)==0x04){delay_us(3);}
			else
			{
				DS3234_code=Read_data_DS3234(0x11);//¸ß×Ö½Ú
				DS3234_code<<=8;
				DS3234_code+=Read_data_DS3234(0x12);//µÍ×Ö½Ú  
				DS3234_code>>=6;//»¹ÓĞ10Î»£¬×î¸ßÎ»¸ººÅÎ»
				delay_us(1);
				if(DS3234_code<512)//2µÄ9´ÎÃİ ÎªÕı 511
				{
					tempcode=DS3234_code;
					tempcode>>=1;//¼õĞ¡1Î»¾«¶È  /?
					temp_neg_flag=0;
				}
				else//Îª¸º
				{
					DS3234_code=(~DS3234_code);//È¥·´ 
					DS3234_code&=0x7f;
					DS3234_code+=1;//·Ç¸ºÊı È¥·´¼Ó1
					tempcode=DS3234_code;
					tempcode>>=1;
					disbuf[31]='-';
					temp_neg_flag=1;//Õı¸º±êÖ¾Î»
				}
			}
		}//*/
		/*****************************Ê±¼äĞŞ¸Ä*********************************/
		if(Alter_time_flag==1) //±ê¶¨Ê±¼ä
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
	   /*****************************ÎªÊ²Ã´£¿*********************************/  //ÔÚÔ­³ÌĞòÀïÊµÏÖÖĞ¶ÏÇ¶Ì×
// 	     USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//´®¿Ú3½ÓÊÕÊ¹ÄÜ//´®¿Ú1½ÓÊÕÖĞ¶ÏÊ¹ÄÜ¿ª
// 	     EXTI->IMR |= EXTI_Line15;//Ê¹ÄÜÍâ²¿ÖĞ¶Ï15;//¿ªÍâ²¿ÖĞ¶Ï AD7730ÖĞ¶Ï
// 	     SEI();//´ò¿ªÈ«²¿ÖĞ¶Ï
	   /*****************************Ê±¼ä¼ÇÂ¼±êÖ¾Î»*********************************/
		 if(ms_secb>=10){ms_secb=0;} //³ıÆ¤±ê¶¨¼ÆÊ±  ¸Ä
	     ms_secb+=1; //Ã¿100ms+1   2sÊ±ÇåÁã 
	   /**************************** Ë¢ĞÂÏÔÊ¾*********************************/
		 dis_play();
           /**************************** AD7730ÂëÖµ´¦Àí*********************************///ÓÃÁË16Î»¾«¶È
		if(run==0)  //×ª×Ó³ÓÃ»ÓĞÔËĞĞÊ±£¬ĞèÒªÊı¾İÏÔÊ¾¸ººÉ£¬Ã»ÓĞÔËĞĞÊ±µÄ¸ººÉÁ¿Îª£¨Ö±½Ó²âÁ¿-Æ¤ÖØ£©Ã»ÓĞÍ¨¹ı¸Ü¸Ë±ÈµÄ×ª»¯
		{		
			ad_flag=1;//¿ØÖÆADÖĞ¶ÏµÄ¶ÁÈ¡
			for(i=0;i<26;i++)
			{
				ad7730_code_result[i]=ad7730_filter[i];  //Êı¾İ×ªÒÆ
			}
			qsort(ad7730_code_result,26,sizeof(uint32_t),comp);//¶ÔÊı¾İÅÅĞò  ´ÓĞ¡µ½´óÅÅĞò
			addatab=0;
			for(i=5;i<21;i++)
			{
				addatab+=ad7730_code_result[i];//5µ½20Ïà¼Ó  16¸öÊı 
			}
			addatab>>=12;//×óÒÆ12Î»£¬³ıÒÔ4096,£¿×óÒÆ4Î»£¬³ıÒÔ16£¬×óÒÆ8Î»³ıÒÔ256//È¥ÁË8Î»¾«¶È //³ıÒÔ16£¬ÇóÈ¡ÂëÖµµÄÆ½¾ùÖµ
			addat=addatab;//16Î»ÂëÖµ 
			ad_flag=0; //¿ØÖÆADÖĞ¶ÏµÄ¶ÁÈ¡
			sumquality=addat*presspra-tarevalue;//sumqualit Îª³ÆÖØ´«¸ĞÆ÷Ö±½Ó²âÁ¿µÄÃ»ÓĞÍ¨¹ı¸Ü¸Ë±ÈµÄ×ª»¯
	  }
		/**************************** ¼ì²éËø»úÊ±¼äÊÇ·ñÒÑµ½,Èç¹ûµ½ÁË¾ÍÍË³ö*********************************/
		if(xram[125]==1)
	  {
		 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾ ²»Çå³ı±êÖ¾Î»£¬½ø²»ÁËÖĞ¶Ï
		 return;//¼ì²éËø»úÊ±¼äÖÃÎ»  ÖØÒª ÖØÒª 
    }			
		/**************************** ¼ÆÊ±±êÖ¾Î»*********************************/
		if(hsec>=10)//1.0s   1sÑ­»·Ò»´Î
		{
			
			hsec=0;
			ms_secb=0; 
			tx0_sec=0;//dissignalÓÃ   
			sec=sec+1;//ºÍ±ê¶¨³ÌĞòÓĞ¹Ø
			if(rjtime)rjtime--;//¸Õ¿ªÊ¼Æô¶¯Ê±£¬Èİ»ı¹¤×÷·½Ê½
			else{vmode=0;}//      
			if(run)//¼ì²âËÙ¶È´«¸ĞÆ÷ÓÃ
			{
				if(running<200)running++; //1s¼ì²éÒ»´Î   £¨ÊÂ¼şĞÅÏ¢ÓÃ£©
				if((speedcode<3)&&xram[9])speedless++;//ËÙ¶È´«¸ĞÆ÷ÓĞĞ§  P9ËÙ¶È´«¸ĞÆ÷ÊÇ·ñÎªÓĞĞ§¡£ 0ËÙ¶È´«¸ĞÆ÷ÎŞĞ§£¬ËÙ¶ÈÓÉP5¾ö¶¨¡£1ËÙ¶ÈÓÉ´«¸ĞÆ÷ÊäÈë
				else speedless=0;//(ËÙ¶È´«¸ĞÆ÷ÎŞĞ§)»òÊÇ(ËÙ¶È´«¸ĞÆ÷ÓĞĞ§²¢ÇÒspeedcode>3)
			}
			else {speedless=0;running=0;}//³ÓÃ»ÓĞÔËĞĞ
			if(s5_flag);//ÃÜÂëÓĞĞ§±êÖ¾Î»  
			else if(s5)  //ÆÕÍ¨ÃÜÂë
			{
				s5_second--;
				if(!s5_second)s5=0;
			}
			if(s6_flag);
			else if(s6)  //³¬¼¶ÃÜÂë
			{
				s6_second--;
				if(!s6_second)s6=0;
			}
		}
		/*********************************´®¿Ú1¶¨Ê±·¢ËÍÊı¾İ*****************************/
		if(usart_send_time>=50)
		{
			usart_send_time=0;
			if(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			{
		   switch(tx0_count)//·¢ËÍ¼ÆÊıÆ÷  //·¢ËÍ Ê±¼ä  1°àÀÛ¼Æ 2°àÀÛ¼Æ 3°àÀÛ¼Æ ×ÜÀÛ¼Æ
			 {
				case 0:USART_SendData(USART1, prn_time[0]);tx0_count++;break;//´òÓ¡Ê±¼ä  Äê     
				case 1:USART_SendData(USART1, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART1, '-');tx0_count++;break;
				case 3:USART_SendData(USART1, prn_time[2]);tx0_count++;break;          //ÔÂ
				case 4:USART_SendData(USART1, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART1, '-');tx0_count++;break;	
				case 6:USART_SendData(USART1, prn_time[4]);tx0_count++;break;          //ÈÕ
				case 7:USART_SendData(USART1, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART1, 0x0d);tx0_count++;break; //»Ø³µ
				case 9:USART_SendData(USART1, 0x0a);tx0_count++;break; //»»ĞĞ
				case 10:USART_SendData(USART1, prn_time[6]);tx0_count++;break;         //Ê±
				case 11:USART_SendData(USART1, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART1, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART1, prn_time[8]);tx0_count++;break;         //·Ö
				case 14:USART_SendData(USART1, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART1, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART1, prn_time[10]);tx0_count++;break;        //Ãë
				case 17:USART_SendData(USART1, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART1, 0x0d);tx0_count++;break;//»Ø³µ Enter
				case 19:USART_SendData(USART1, 0X0a);tx0_count++;break;//»»ĞĞ
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
				case 42:USART_SendData(USART1, prnbuf1[7]);tx0_count++;break;//prnbuf1  1°à×éÀÛ¼ÆÁ¿
				case 43:USART_SendData(USART1, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART1, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART1, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART1, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART1, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART1, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART1, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART1, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART1, 0x0d);tx0_count++;break;//»Ø³µ»»ĞĞ
				case 52:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART1, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART1, 'H');tx0_count++;break;
				case 55:USART_SendData(USART1, 'I');tx0_count++;break;
				case 56:USART_SendData(USART1, 'F');tx0_count++;break;
				case 57:USART_SendData(USART1, 'T');tx0_count++;break;
				case 58:USART_SendData(USART1, ':');tx0_count++;break;
				case 59:USART_SendData(USART1, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART1, 0x0d);tx0_count++;break;//»Ø³µ
				case 61:USART_SendData(USART1, 0x0a);tx0_count++;break;//»»ĞĞ
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
				case 84:USART_SendData(USART1, prnbuf2[7]);tx0_count++;break;//prnbuf2  °à×é2
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
				case 126:USART_SendData(USART1, prnbuf3[7]);tx0_count++;break;//prnbuf3  °à×é3
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
				case 168:USART_SendData(USART1, prnbuf4[7]);tx0_count++;break;//prnbuf4 ×ÜµÄÀÛ¼ÆÁ¿ 
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
				case 193:USART_SendData(USART1, 0x0d);tx0_count++;break;//»Ø³µ Enter	
				case 194:USART_SendData(USART1, 0x0a);tx0_count++;break;//»»ĞĞ
				case 195:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 196:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 197:USART_SendData(USART1, 0x0d);tx0_count++;break;
				case 198:USART_SendData(USART1, 0x0a);tx0_count++;break;
				case 199:tx0_count=0;break;
			}
		}
	}
		 /**************************** ÌØÊâÇé¿öµÄÏÔÊ¾£¨ÊÂ¼ş±¨¾¯ÏÔÊ¾£©*********************************/
		  if(t_tmp>=1) //++ÔÚ¶¨Ê±ÖĞ¶Ï¿ªÊ¼   Ã¿100msÖ´ĞĞÒ»´Î
		  {
			t_tmp=0; //++ÔÚ¶¨Ê±ÖĞ¶Ï¿ªÊ¼
			tx0_sec++;//
			hsec++;
			usart_send_time++;
			if(disrunflg)//ºÜ¶àº¯Êı°üº¬´Ë±êÖ¾Î»
			{
				/////////////////Èİ»ı¼üÅÌÄ£Ê½Í¼±êÏÔÊ¾///////////////////
				if((avmode==1)&&(keymode==1))//¼üÅÌºÍÈİ»ıÄ£Ê½ 
				{ 
					disbuf[0]=0xa1;
					disbuf[1]=0xf8;
				}
				else if((avmode==0)&&(keymode==1))//¼üÅÌÄ£Ê½
				{
					disbuf[0]=0xa1;
					disbuf[1]=0xf7;
				}
				if(keymode==0)//
				{
					disbuf[0]=0x20;//²»ÏÔÊ¾
					disbuf[1]=0x20;
				}	
				
				ls_sec=(uint32_t)sec;
				i=ls_sec%2;//ÆæÊıÃë
				
				/////////////////ÅúÁ¿Éú²úÍ¼±êÏÔÊ¾///////////////////
				if(run&&i)//ÔÚÔËĞĞ×´Ì¬ÇÒsecÎ»ÆæÊıÊ±
				{
					if(bflag==0) //Ã»ÓĞÅúÁ¿Éú²ú
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xfa;
					}
					else //ÅúÁ¿Éú²ú
					{
						disbuf[2]=0xa1;
						disbuf[3]=0xc6;
					}
				}
				else//secÎªÅ¼Êı »òÊÇÃ»ÓĞÔËĞĞÊ±
				{
					if(run&&bflag)//ÔËĞĞÇÒÅúÁ¿Éú²ú±êÖ¾Î»1
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
				
			  /////////////////·ÇÈİ»ıÍ¼±êÏÔÊ¾£¨ÖØÁ¿¹¤×÷·½Ê½¸ÕÆô¶¯£©///////////////////
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
				
				/////////////////±¨¾¯ÊÂ¼şÍ¼±êÏÔÊ¾///////////////////
				if(warnflag&&!i)//±¨¾¯±êÖ¾Î»1  ÉÁË¸ÓÃ
				{
					disbuf[20]=0x20;
					disbuf[21]=0x20;
				}
				else
					warn();//ÊÂ¼ş±¨¾¯ÏÔÊ¾
			  }
		   }
		 /**************************** ÀÛ¼ÆÂö³åÊä³ö*********************************/
		 if(pulsecount)
		  pulsecount--;  //Êä³öÂö³åµÄÊ±¼ä //Ê±¼ä¼ÆËãÓĞÎÊÌâpulsecount 
 	    else
			{
				dout(6,0);   //ÀÛ¼ÆÂö³åÊä³ö
			}
	    /**************************** ±¨¾¯ÊÂ¼ş´¦Àí*********************************/

      if(c1||h7||h4||l4||((xram[41]==1)&&h9))//C1³ÆÖØ´«¸ĞÆ÷ÊäÈë´íÎó//h7µç»ú¹ÊÕÏ//h4³ÆÖØ´«¸ĞÆ÷¸ºÔØ¹ı´ó//l4³ÆÖØ´«¸ĞÆ÷¸ºÔØÌ«Ğ¡ ´ı¸Ä  ³öÏÖ¹ÊÕÏÊ±²»Êä³ö±¸Í× plc³ÓÆô¶¯ĞÅºÅ¹Ø±Õ 
			{
				ready_flag=1;//±¸Í×Êä³ö±ê¼Ç  //ÏÂ±ßÓÃ      //P41 Æ¤´øÅÜÆ«¿ØÖÆÑ¡Ôñ 1 ¶Ë×ÓX6£º5£¬6¶Ï¿ªÊ±£¬ÒÇ±í¿ØÖÆÆ÷µçÁ÷»·Í£Ö¹//h9Æ¤´øÅÜÆ«»òÏÖ³¡¿ØÖÆ 
				if(flash_light==1)dout(10,1);//¹ÊÕÏÖ¸Ê¾µÆ±¨¾¯ÉÁË¸
				else dout(10,0);
				dout(2,1);//Êä³ö±¨¾¯ĞÅºÅ DO3±¨¾¯Êä³ö ¼ÌµçÆ÷Êä³ö±¨¾¯
			}
			else//ÉÏÊöÊÂ¼ş¶¼Ã»·¢ÉúÊ±£¬
			{
				ready_flag=0;
				dout(10,0);//±¨¾¯¹ÊÕÏÖ¸Ê¾µÆ¹Ø£¬±¨¾¯ĞÅºÅ¹Ø
				dout(2,0); //¼ÌµçÆ÷Êä³ö±¨¾¯
			}
			////////±¸Í×¼ì²âÊä³ö///////
			if(!keymode)//·Ç¼üÅÌÄ£Ê½  ĞÂ¶¨ÒåµÄDO3Îª±¨¾¯Êä³ö(Ó°ÏìÍ£³µµÄ±¨¾¯) DO8¶¨ÒåÎª±¸Í×  ·Ç¼üÅÌÄ£Ê½Ê±£¬±¸Í×²»Êä³ö¸øPLC£¬µ¼ÖÂPLCÆô¶¯ĞÅºÅ²»Êä³ö£¬´Ó¶øÒÇ±í³ÓÍ£Ö¹
			{
			///	if(!c1)dout(2,1);//¹Ø¼üÅÌ¡¢ÎŞAD±¨¾¯ÔòÊä³ö±¸Í×¡£
			///	else dout(2,0);
				if((xram[46]==0||xram[46]==2)&&(!ready_flag)) dout(8,1);//ÎŞ±¨¾¯,ÔòÊä³ö±¸Í×ĞÅºÅ //P46=0 Æô¶¯Îª¿ª¹ØÁ¿X3:3,4;Éè¶¨ÖµÖµÓÉÄ£ÄâÁ¿X6£º1£¬2  ĞŞ¸Ä¹ı p46=0ºÍP46=2µÄÇé¿ö¶¼Òª¿¼ÂÇ
				else dout(8,0);//±¨¾¯Ê±£¬²»Êä³ö±¸Í×
				if((xram[43]==1)&&(xram[46]==1)) status[0]=0x31;//P43=1 ÓĞ´®ĞĞÍ¨Ñ¶ =0ÎŞ´®ĞĞÍ¨Ñ¶ p46=1 Æô¶¯¼°Éè¶¨ÖµÀ´Ô´ÓÉ´®¿ÚÊä³ö ´®¿ÚÏòÉÏÊä³ö±¸Í×ĞÅÏ¢//´ı¸ÄÓĞÎÊÌâ
				else status[0]=0x30;
				
			}
			else//¼üÅÌÄ£Ê½  //¼üÅÌÄ£Ê½ÎŞĞèÊä³ö±¸Í×ĞÅºÅ
			{
			///	dout(2,0); 
				dout(8,0);//¿ª¼üÅÌ¹Ø±¸Í×
				status[0]=0x30; //Éè¶¨ÖµÓÉ¼üÅÌÊäÈë£¬Æô¶¯ºÍÍ£Ö¹Ò²À´×Ô¼üÅÌ
			}
			if((dis_dissigal_flag==1)&&(menu_exit_flag==1)) //Ğí¶àº¯ÊıÓÃµ½ÁË //ÕâĞ©±êÖ¾Î»¿ØÖÆÄ³Ğ©º¯ÊıµÄÖ´ĞĞÊ±¼ä
			{
				dis_sec_count++;
				if(dis_sec_count>=2) //200ms
				{
					dis_dissigal_flag=0; //ÔÚÖ÷ÏÔÊ¾º¯ÊıÓÃ
					menu_exit_flag=0;
					dis_sec_count=0;
				}
			}
			if((xram[43]==1)&&(M485REI ==1)&&(M485DEI==1))//P43Î»1 ÓĞ´®¿ÚÍ¨Ñ¶ //RE =1(PJ4),DE =1(PJ3) ´ËÊ±·¢ËÍÄ£Ê½     //Ê¹ÄÜRE=0 DE =1  485·¢ËÍ3s,½ÓÏÂÀ´ÊÇ½ÓÊÜ ¡£485½ÓÊÕºÍ·¢ËÍÆµÂÊÎª3s
			{
				RE_DE_Count++;
				if(RE_DE_Count==30)
				{
					USART_ITConfig(USART3, USART_IT_TC, DISABLE);//disable TX ¹Ø±Õ·¢ËÍ        //¹Ø±Õ·¢ËÍÍê³ÉÖĞ¶Ï
					USART3->CR1|=1<<2;    //½ÓÊÕ»º³åÇø·Ç¿ÕÖĞ¶ÏÊ¹ÄÜ;		//enable RX  ´ò¿ª½ÓÊÜ£¨ÕâÀï´ò¿ª£¬ÔÚÄÄÀï¹Ø±ÕÁË½ÓÊÕ£©   //¹Ø±ÕµÄÊÇ·¢ËÍ  
					USART_ClearFlag(USART3,USART_SR_RXNE);
					M485REI =0;			//RE enable ;DE disenable // RE =0  DE=0 /¿ÉÒÔ½ÓÊÕ²»ÄÜ·¢ËÍ
                    M485DEI =0;					
				}
			}
			else RE_DE_Count=0;			
			/*************************************±£»¤Í£Ö¹¿ØÖÆ****************************************/
// 			if((!din(2))&&(run)&&(keymode)&&(xram[46]==0)) stop();//DI3ÊÍ·Å¿ª¹Ø´ò¿ª//ÔËĞĞ//¼üÅÌÄ£Ê½//P46=0:Æô¶¯ºÍÉè¶¨Öµ¾ùÀ´×Ô¶Ë¿Ú £¨ÓĞÎÊÌâ£©´ı¸Ä ÊÇ·ñĞèÒªÅĞ¶ÏP46 
			if((!din(2))&&(run)&&(keymode)) stop();//¸Ä¹ı£¬ÓëÉÏÒ»Óï¾ä¶ÔÕÕ
			if((!din(2))&&(din(1))&&(run)&&(!keymode)&&((xram[46]==0)||(xram[46]==2))) stop(); //run=0;ctrlout=0;running=0;enablerun=0;ek=0;ek1=0;  DI3  DI2
			if((run)&&((c1)||(c2)))
			 stop();//ÔËĞĞ////³ÆÖØ´«¸ĞÆ÷ÊäÈë³ö´í////ËÙ¶È´«¸ĞÆ÷ÊäÈë³ö´í   ´®¿ÚÊ±»¹Í£Âğ£¿ ÎÊÌâ
			/*************************************·Ç¼üÅÌÄ£Ê½ÆôÍ£¿ØÖÆ£¨ÆôÍ£¶Ë¿Ú¿ØÖÆ£©****************************************/
			if((!run)&&din(1)&&din(2)&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//¹Ø±Õ//DI2 Æô¶¯ºÍÍ£Ö¹//DI3±ÕºÏ//Í£Ö¹¼üÅÌÄ£Ê½//Æô¶¯ÓÉ¶Ë¿Ú£¬Éè¶¨ÖµÓÉ¼üÅÌ»òÊÇ¶Ë¿Ú   DI3  DI2
			{
				start();//Æô¶¯¿ª¹ØÁ¿ÊäÈëºÍ4-20mAÊä³ö   // 1.·Ç¼üÅÌÄ£Ê½  Æô¶¯À´×Ô¶Ë¿Ú   
			}
			if((run)&&(!din(1))&&(!keymode)&&((xram[46]==0)||(xram[46]==2)))//ÔËĞĞ//DI2Æô¶¯ºÍÍ£Ö¹//Í£Ö¹¼üÅÌÄ£Ê½//Æô¶¯ÓÉ¶Ë¿Ú£¬Éè¶¨ÖµÓÉ¼üÅÌ»òÊÇ¶Ë¿Ú  °´ÁË³ÓÍ£Ö¹¼ü
			{
				stop();//Æô¶¯¿ª¹ØÁ¿ÊäÈëºÍ4-20mAÊä³ö   
			}
			/***************************************¼üÅÌÄ£Ê½Í£Ö¹Æô¶¯*******************************************/
			  if(keymode==1)//¼üÅÌÄ£Ê½
			 {   mhang0=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_0); //¶ÁÈ¡ĞĞ
				 mhang1=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_1); 
				 mhang2=GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_2); 
				 hang_1=1;//
				 hang_2=1;//
				if(run)//Èç¹ûÔÚÔËĞĞ×´Ì¬
				{
					hang_0=0;
					if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)
					{
						delay_us(10);
						if((GPIO_ReadInputData(GPIOE)&0X00FF)==0xfe)stop();
					}
				}
				else//ÔÚÍ£Ö¹Ä£Ê½Ê±
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
        /**************************** CPUÍâ¸ø¶¨ÂëÖµµÄ´¦Àí*********************************/
			anainput=flow_set();//Íâ¸ø¶¨  ·µ»ØÂëÖµ   ´¦Àí·½Ê½´ı¸Ä
      if((!keymode)&&(xram[46]==0))//ÏµÍ³½øÈëÍ£Ö¹¼üÅÌ·½Ê½£¬0 Éè¶¨ÖµµÃÀ´Ô´ÓÉÄ£ÄâÁ¿X6£º1£¬2ÊäÈë    Æô¶¯Îª¿ª¹ØÁ¿X3£º3£¬4   //CPUÍâ¸ø¶¨Éè¶¨ÖµÁ÷Á¿µÄ´¦Àí
			{
				lsvalue=(anainput-settare)*setpra*input_rate;//input_rate ÊÖ¶¯ÊäÈë//µçÁ÷¶ÔÓ¦µÄ¶ÖÊı setpraµ¥Î»ÂëÖµ¶ÔÓ¦µÄÁ÷Á¿£¨t/h£© input_rate(ÊÖ¶¯ÊäÈë±ÈÀıÖµ)
				if(lsvalue>0)//Á÷Á¿ÖµÂË²¨
				{
					setvalue=setvalue_1+ap*(lsvalue-setvalue_1);//ap=0.5/P54  P54Á÷Á¿ÊäÈëÂË²¨Ê±¼ä
					setvalue_1=setvalue;
				}
				else setvalue=0;
			}
			/*************************************´®¿ÚÁ÷Á¿¸ø¶¨ÆôÍ£´¦Àí****************************************/
			if((xram[43]==1)&&(xram[46]==1)&&(!keymode)&&(r_flag=='Y'))//P43ÓĞ´®¿ÚÍ¨ĞÅ//Æô¶¯ºÍÉè¶¨Öµ¾ùÀ´×Ô´®¿ÚÊäÈë//Í£Ö¹¼üÅÌ  //´®¿ÚÓÃ
			{
				if((lsset=ser_setvalue())!=-1)//´®¿Ú¸ø¶¨
					setvalue=lsset;//´®¿ÚÁ÷Á¿¸ø¶¨
				if((rs_temp[3]=='K')||(rs_temp[3]=='k'))
				{
					if((rs_temp[2]==0x31)&&(run==0)) //2 Æô¶¯ºÍÍ£Ö¹  //2.·Ç¼üÅÌÄ£Ê½ ÆôÍ£À´×Ô´®¿Ú
						start();
					else if((rs_temp[2]==0x30)&&(run==1))
						stop();
				}
			}
	
    /*******************************Á÷Á¿µÄ¼ÆËã´¦Àí*******************************************/  //Á÷Á¿¾ùÖµµÄÇóÈ¡

			if(run)//ÔËĞĞ×´Ì¬ÏÂ£¬¼ÆËãÁ÷Á¿Öµ
			{   
				ctrealvalue=press*speedvalue*3.6*xram[36];//t/h   Á÷Á¿ÀÛ¼ÆÓÃ
				realvalue=pressvalue*speedvalue*3.6*xram[36];//pressvalueÔØºÉ //p36ÊµÎï±ê¶¨µ÷ÕûÏµÊı£ºp36=p36*Êµ¼ÊÀÛ¼ÆÖµ/ÏÔÊ¾ÀÛ¼ÆÖµ  Á÷Á¿Öµ£¨ÎªÇóÏÂ±ßÁ÷Á¿Öµ£© kg/¶È* ¶È/s *3.6 =t/h(·´À¡Á÷Á¿Öµ)   ÏÔÊ¾ÓÃ
				lsreal=Xn*speedvalue*3.6*xram[36];  //p36ÊµÎï±ê¶¨µ÷ÕûÏµÊı£ºp36=p36*Êµ¼ÊÀÛ¼ÆÖµ/ÏÔÊ¾ÀÛ¼ÆÖµ  £¨µ²°å¿ØÖÆ£© kg/¶È* ¶È/s *3.6 =t/h £¨½øÈëÁ÷Á¿Öµ¿ØÖÆµ²°å£©
			}
			else {ctrealvalue=0,realvalue=0,lsreal=0;}//Í£Ö¹Ê±Á÷Á¿ÇåÁã
			
			////////////////////ÏÔÊ¾ÓÃ////////////////////
			real=real_1+ai*(realvalue-real_1);//Á÷Á¿ÖµÂË²¨
			real_1=real;
			for(i=1;i<20;i++)avernum[i-1]=avernum[i]; //Êı×é×óÒÆ
			avernum[20-1]=real;//ak=30
			tracevalue=0;
			for(i=0;i<20;i++)tracevalue+=avernum[i];//ÇóÈ¡Æ½¾ùÖµ
			tracevalue/=20;//ÏÔÊ¾ÓÃ Á÷Á¿·´À¡
			delay_us(1);

     /*******************************µ²°å¿ØÖÆµ÷½Ú*******************************************/ 
     if((run)&&(!calflg))//ÔËĞĞ²¢ÇÒ·Ç±ê¶¨Ä£Ê½  
    {
			////////////////////Çı¶¯ºÍÓ¦´ğ////////////////////
	  runflag_count++;
	  if(runflag_count>=5)runflag_count=5;
	  dout(7,1);//³ÓÆô¶¯   ³ÓµÄÇı¶¯ĞÅºÅ ×ª×Ó³Óµã»úÆô¶¯
	  if(!xram[9]){speed_dis=xram[5];}//ËÙ¶È´«¸ĞÆ÷ÎŞĞ§£¬ËÙ¶ÈÓÉP5¾ö¶¨ÉÏ±ßÒÑ¸øspeedvalueÓÃÓÚ¼ÆËã£¬speed_disÓÃÓÚÏÔÊ¾ //ÎŞÓÃ
	  if(xram[9]){if(speedcode>5)dout(0,1);}//1ËÙ¶È´«¸ĞÆ÷ÓĞĞ§£¬speedcode 100msËÙ¶È¼ÆÊıµÄÂëÖµ   DO1=1 ÔËĞĞ×´Ì¬  500msÓĞÁËËÙ¶ÈÖ®ºó£¬(Ó¦´ğĞÅºÅÊä³ö£©   ´ı¸Ä
	  else {if(runflag_count>=5)dout(0,1);} //Ó¦´ğĞÅºÅÊä³ö   //´ı¸Ä
	  if(modecount==1)//¸ÕÆô¶¯¿ØÖÆÄ£Ê½1 ¹Øµ²°å ¿ª×ª×ÓËÙ¶È £¨Ê£ÓàµÄÁÏÂ©ÏÂÈ¥£©
	  {
       ctrloutb=daminb;//¹Øµ²°å
    }
	  else if(modecount==2)  //¿ªµ²°åºÍ×ª×ÓËÙ¶È  ·Ö¸ôµ¥Ôª¿ªÊ¼¼Ç·Ö¸ôµ¥ÔªµÄÖØÁ¿
	  {   
      ctrloutb=xram[73]*boardcal;//P73Èİ»ı·½Ê½Çı¶¯µçÁ÷ 4096/20.88  ×î´óÂëÖµ¶ÔÓ¦µÄ×î´óµçÁ÷//´ı¸Ä   £¨ÒÔÈİ»ı·½Ê½¿ØÖÆµ²°å¿ª¶È£©         				
    }
    else if(modecount==3)  //Èİ»ı·½Ê½£¨ÊÖ¶¯Ä£Ê½£©µ²°åµÄ¿ØÖÆ  ËÙ¶ÈµÄ¿ØÖÆ·ÅÔÚ¼ÆÊıÆ÷Àï
			{   
				if(avmode||(!avmode))//Èİ»ı·½Ê½¡£ÔÚÖØÁ¿¹¤×÷µ±ÖĞ£¬Æô¶¯¿ªÊ¼Ê±£¬ÏÈ°´Ò»ÏÂ¿ØÖÆÒ»¶ÎÊ±¼ä
				{  
					ctrloutb=xram[73]*boardcal;//P73Èİ»ı·½Ê½Çı¶¯µçÁ÷ 4096/22.9  ×î´óÂëÖµ¶ÔÓ¦µÄ×î´óµçÁ÷//´ı¸Ä   
					ctrloutb+=avdanumb;
					if(ctrloutb<=daminb) ctrloutb=daminb;
					if(ctrloutb>=damaxb) ctrloutb=damaxb;
					if(ctrloutb>4095)    ctrloutb=4095;   //Èİ»ı·½Ê½µ²°å¿ª¶È¿ØÖÆ
				}
				else if((xram[38]==1)&&(h5==0)){}//Æ«²î·¶Î§ÄÚ²»µ÷ËÙP38Æ«²î¿ØÖÆÑ¡Ôñ£ºµ±Ñ¡Ôñ0Ê±£¬ÔÚÈÎºÎÊ±¼äÄÚµ÷½Úµã»÷ËÙ¶È¡£µ±Ñ¡Ôñ1Ê±£¬ÔÚÆ«²î·¶Î§ÄÚ²»µ÷½Úµç»úËÙ¶È  ÔÚÆ«²î²»³¬ÏŞÊ±
				else//ÖØÁ¿¹¤×÷·½Ê½                               //Ñ¡Ôñ´Ë²ÎÊı¿É¼õĞ¡µç»úÕğµ´£¬Ê¹µÃÎïÁÏÔËĞĞÆ½ÎÈ¡£ 
				{
	////			ek1=ek;
					if((bflag==0)||(((zb-zi)>0)&&bflag))//ÅúÁ¿Éú²ú±êÖ¾Î»0 bsetend 5*set_value/3600//Á÷Á¿Éè¶¨Öµ  ((zb-zi)>bsetend)&&bflag)  zbÅúÁ¿Éú²úÉè¶¨Öµ ziÅúÁ¿Éú²úÔöÁ¿
					{     
						ek=setvalue-lsreal;//Éè¶¨Öµ-Á÷Èë×ª×Ó³Ó·´À¡Öµ=Æ«²î 
	////			danum=kpi*(ek*ki-ek1);//ek1ÉÏÒ»Ê±¿ÌµÄÆ«²î B/t/h*(t/h-t/h) =B  »ı·ÖÒÖÖÆ³¬µ÷ ki =1+1/P12  Æ¤´ø³Ó
						danumb=(kp+ki+kd)*ek+(-kp-2*kd)*ek1+kd*ek2;//ÔöÁ¿¿ØÖÆ
						ctrloutb+=danumb;
						if(ctrloutb<=daminb)ctrloutb=daminb;//ÂëÖµ×îĞ¡Öµ
						if(ctrloutb>=damaxb)ctrloutb=damaxb;//ÂëÖµ×î´óÖµ
						if(ctrloutb>4095)ctrloutb=4095;
						ek2=ek1;
						ek1=ek;
					}
					else//if((bflag =1)&&((zb-zi)<=bsetend))   //ÅúÁ¿Éú²ú½áÎ²¿ØÖÆ  È¥µô
					{
						if(bctr==0)
						{
							danumb=ctrloutb/220;
							bctr=1;
						}
						ctrloutb-=danumb;//100msµÄ¿ØÖÆÖÜÆÚ  »ºÂıµÄ¼õĞ¡
						if(ctrloutb<(daminb+300)) ctrloutb=daminb+300;
						else if(ctrloutb>damaxb) ctrloutb=damaxb;
						if(ctrloutb>4095)ctrloutb=4095;
					}
			 }
			 
		 }
  }
		else if(!calflg)//if((run==0)&&(!calflg)) //Í£Ö¹ÔËĞĞ
		{
			runflag_count=0;
			dout(0,0);//ÔËĞĞ×´Ì¬¹Ø£¨Ó¦´ğ£©
			dout(7,0);//×ª×Ó³Ó¹Ø±Õ //Æô¶¯  Çı¶¯
			ctrlout=ctrlana1;////Í£Ö¹Êä³ö×îĞ¡ÖµmA
			ctrloutb=ctrlana2;//Í£Ö¹µ²°åÊä³ö×îĞ¡ÖµmA
			if(!xram[9]){speedvalue=0;speed_dis=0;}//ËÙ¶È¿ª¹ØÎŞĞ§Ê±
			plusesum=0;//ÀÛ¼ÆÂö³åÊıÇåÁã
		    modecount=0;//Ä£Ê½0 Ã»Æô¶¯
			plusecount=0;//¼ÆÊıÖĞ¶ÏµÄ¼ÆÁ¿
		}
	/***************************************100msÁ÷Á¿ÇóÈ¡*******************************************/
		if(calflg)//±ê¶¨±ê¼Ç  //Èİ»ı·½Ê½ÔËĞĞ
		{
			total_temp=0;                                                       //P42 0:µ±ºÉÖØÏÂÏŞL2±¨¾¯Ê±£¬ÀÛ¼Æ¼ÆÊıÆ÷¼ÌĞøÀÛ¼Æ//1:µ±ºÉÖØÏÂÏŞL2±¨¾¯Ê±£¬ÀÛ¼ÆÁ¿²»¼ÆÊı
			if((pressvalue<xram[24])&&(xram[42]==1)){delay_us(1);} //ºÉÖØĞ¡ÓÚºÉÖØÏÂÏŞÓëµ±ºÉÖØÏÂÏŞL2±¨¾¯Ê±£¬ÀÛ¼Æ¼ÆÊıÆ÷²»ÀÛ¼Æ
			else total_cal+=ctrealvalue*0.000027777778*xram[25];//ÇóÀÛ¼ÆÁ¿//Ã¿100msÀÛ¼ÆÒ»´ÎÊıÖµ//p25 =p25(Ô­Öµ)*³ÆÁ¿Êµ¼ÊÖµ/ÏÔÊ¾ÀÛ¼ÆÖµ    ×¢£º£¨ÏÔÊ¾ÀÛ¼ÆÖµ£©²âÁ¿ÀÛ¼ÆÖµ  0.000027 =1/36000
		}
		else if((l2==1)&&(xram[42]==1))//ºÉÖØĞ¡ÓÚºÉÖØÏÂÏŞÓëµ±ºÉÖØÏÂÏŞL2±¨¾¯Ê±£¬ÀÛ¼ÆÁ¿ÇåÁã  //1ÀÛ¼ÆÁ¿²»ÀÛ»ı
			{total_temp=0;}  
		else
			{total_temp=ctrealvalue*0.000027777778*xram[25];} //p25 =p25(Ô­Öµ)*³ÆÁ¿Êµ¼ÊÖµ/ÏÔÊ¾ÀÛ¼ÆÖµ×¢£º£¨ÏÔÊ¾ÀÛ¼ÆÖµ£©²âÁ¿ÀÛ¼ÆÖµ£¨Õı³££©//1h=3600s=36000£¨100ms£©

/***************************************ÀÛ¼ÆÁ¿*******************************************/
	    z1_temp+=total_temp;//total_temp 100msµÄÀÛ¼ÆÖµ//total_tempÁ÷Á¿Öµ t  //ÀÛ¼ÆµÄÁ÷Á¿Öµ
// 			z1 =0;
// 			z1_temp =0;
// 			totalshift[0]=0;
// 			totalshift[1]=0;
// 			totalshift[2]=0;
			
		if(fabs(z1_temp)>=xram[26])//¼ÆÊıÆ÷Âö³åµ¥Î»
		{
			if(z1_temp>0)
			{
				z1+=xram[26];//¼ÆÊıÆ÷Âö³åµ¥Î»µ¥Î»ÖØÁ¿
				totalshift[shift]+=xram[26];//¼ÆÊıÆ÷Âö³åµ¥Î» °àÀÛ¼Æ  °à×éÀÛ¼Æ
				z1_temp-=xram[26];
			}
			else//Ê²Ã´Çé¿öÏÂ³öÏÖ
			{
				z1-=xram[26];
				totalshift[shift]-=xram[26];//¼ÆÊıÆ÷Âö³åµ¥Î» °àÀÛ¼Æ  
				z1_temp+=xram[26];		
			}
			if(npulse)dout(6,1);//ÀÛ¼ÆÁ¿//npulse=(uchar)(pramc[27]/100+0.9);//¼ÆËãÀÛ¼ÆÁ¿Âö³åÊä³öÊ±¼ä
			pulsecount=npulse;	//Ê±¼äµ½ dout(6,0)	
		}
		if((bflag==1)&&run)//ÅúÁ¿³É²ú±êÖ¾Î»  //ÅúÁ¿³É²ú¼ì²âÍ£Ö¹
		{
			zi+=total_temp;//zi =0  ÔöÁ¿
			zd-=total_temp;//zd =0  ¼õÁ¿
			if(zi>=zb) //ÔöÁ¿´óÓÚÉè¶¨Öµ ÔòÊä³öÍ£Ö¹
			stop();	
		}

		if(clearflag1)//×Ü¶ÖÎ»ÇåÁã
		{
			totalshift[shift]+=z1_temp;
			z1=0;//×ÜÀÛ¼ÆÁ¿
			z1_temp=0;//ÀÛ¼Æ»º³åÇø
			clearflag1=0;

		}
		if(z1>=9999999){z1-=9999999;}//

    //////////////////////////////////////////////////////////////////////
	if(!calflg){analogout();}//Ã»ÓĞÔÚ±ê¶¨³ÌĞòÊ±Ö´ĞĞ  µ±Ç°Ä£Äâ·´À¡Êä³ö  //Ö´ĞĞ±ê¶¨Ê±£¬Ã»ÓĞÖ´ĞĞ
	
	if(mainboardflagb==0)//µ²°å¿ØÖÆµ÷½Ú
    {
	   DAC7612_2(0,ctrloutb);
    }//µ²°å¿ØÖÆ
	 
  if(mainboardflag==0&&deputyboardflag==0)//µ±Ö÷°åÁ÷Á¿Êä³öĞ£×¼Ê±£¬´Ë±êÖ¾Î»ÖÃ1 ²»¿ÉÒÔÊä³ö
   {
	    DAC7612(0,ctrlout);
   }//ËÙ¶È¿ØÖÆÊä³ö   
	change_shift();//»»°à È·¶¨ÄÇ¸ö°à 
// 	cli();//¹ØÖĞ¶Ï                                  ÓĞÎÊÌâ  //´ı¸Ä
// 	TIMSK1=(1<<OCIE1A);//bit1 Êä³ö±È½ÏÖĞ¶ÏÊ¹ÄÜÎ»¡£   ÓĞÎÊÌâ
	
	 /////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ»¹Ô­////////
	  SysTick->LOAD=t_val;
	  SysTick->VAL=0x00;
	  SysTick->CTRL=t_ctrl;

   } 	 	
	   
} 	 	


/*********************************Íâ²¿ÖĞ¶Ï£¨AD7730¶Á£©********************************/
void EXTI15_10_IRQHandler(void)
{
// 	delay_ms(10);//Ïû¶¶
   if( EXTI_GetITStatus( EXTI_Line15)!=RESET)
   { 
		uint8_t i=0;
		Timedata=0;
		c4=0;
		/////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ±¸·İ////////   
		e_val=SysTick->VAL;//²»Îª0 Ôò 16Î» Îª0 Îª0 16Î»ÇåÁã
		if(e_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//¹Ø±Õ¼ÆÊıÆ÷1
		e_ctrl=SysTick->CTRL;  
			 
		if(ad_flag==0)//ad±êÖ¾Î»
		{
				if(INT7_flag==0)//ÖĞ¶Ï±êÖ¾Î»
				{
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//Êı×éÏò×óÒÆÎ»//×îµÍÎ»Êı×é±»¸²¸Ç
					buf7730=AD7730_READ();//¶ÁÖØÁ¦Öµ
					ad7730_filter[25]=buf7730;//¸øµ½ÂË²¨Æ÷
				}
				else
				{
					INT7_flag=0;
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//Êı×éÏò×óÒÆÎ»
					ad7730_filter[25]=buf7730;//ÉÏÒ»´Î³öÀ´µÄÖµ
					for(i=0;i<25;i++)ad7730_filter[i]=ad7730_filter[i+1];//Êı×éÏò×óÒÆÎ»
					buf7730=AD7730_READ();
					ad7730_filter[25]=buf7730;
				}
		}
		else//ÔÚ¼ÆËãË¢ĞÂ³ÌĞòÊ±£¬Ò»Ö±¼ÆËãÏÂ´Î´úÂë£¬Ö»ÄÜ±£ÁôÒ»´Î²»¶ªÊ§
		{
					INT7_flag=1;
					buf7730=AD7730_READ();
		}	
	/////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ»¹Ô­////////
		 SysTick->LOAD=e_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e_ctrl;
	
		 EXTI_ClearITPendingBit(EXTI_Line15);  //Çå³ıLINE15ÉÏµÄÖĞ¶Ï±êÖ¾Î»
  }	   
}

/*********************************´®¿Ú1ÖĞ¶Ï·şÎñ³ÌĞò£¨½ÓÊÕ¡¢·¢ËÍ£©********************************/
//void USART1_IRQHandler(void)
//{
//	u8 TEXT_TO_SEND[]={"tongxunxieyichengong"};
//	
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)//¼ì²éÖ¸¶¨µÄUSARTÖĞ¶Ï·¢ÉúÓë·ñ	
//	{
//		  uint8_t i=0,j=0;
//		  for(i=14;i>0;i--)//16¸öÊı×é 
//	  	{USART_RX_BUF[i]=USART_RX_BUF[i-1];}//Êı×éÏòÓÒÒÆ¶¯ 
//		  USART_RX_BUF[0]=USART_ReceiveData(USART1); //½ÓÊÕ
//    	if(((USART_RX_BUF[14]=='F')||(USART_RX_BUF[14]=='f'))&&((USART_RX_BUF[10]=='T')||(USART_RX_BUF[10]=='t'))&&((USART_RX_BUF[2]=='N')||(USART_RX_BUF[2]=='n')))//Ã¿Ò»´®Êı¾İ½ÓÊÕÍê³É  
//			{
//			  USART1->CR1|=0x0008;//Ê¹ÄÜ·¢ËÍ
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
/*********************************´®¿Ú3ÖĞ¶Ï·şÎñ³ÌĞò£¨½ÓÊÕ¡¢·¢ËÍ£©********************************/
void USART3_IRQHandler(void)                	//´®¿Ú3ÖĞ¶Ï·şÎñ³ÌĞò
{
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //½ÓÊÕÖĞ¶Ï(½ÓÊÕµ½µÄÊı¾İ±ØĞëÊÇ0x0d 0x0a½áÎ²)
  {
	  uint8_t i=0;
	  float series_ls=0;
		for(i=11;i>0;i--)//12¸öÊı×é 0:Ç°ÃæÈ«²¿Êı¾İ¶ş½øÖÆÀÛ¼ÓºÍ 1£ºÒÇ±íµØÖ·(16½øÖÆ) £¨0,1¶¼ÊÇ¶ş½øÖÆÂë£© 2£º0 1 Æô¶¯ºÍÍ£Ö¹ÃüÁî
		{rs_temp[i]=rs_temp[i-1];}//Êı×éÏòÓÒÒÆ¶¯  //3:"K" 4-10:7Î»Á÷Á¿Éè¶¨Öµ   11:"S"  
		rs_temp[0]=USART_ReceiveData(USART3);//(USART3->DR);//½ÓÊÕ
		if(((rs_temp[11]=='S')||(rs_temp[11]=='s'))&&((rs_temp[3]=='K')||(rs_temp[3]=='k')))//Ã¿Ò»´®Êı¾İ½ÓÊÕÍê³É  
		{
			if(rs_temp[1]!=xram[44])return;//P44´®ĞĞÍ¨Ñ¶µØÖ·£ºÔÚÒ»×é×ÜÏßÖĞ£¬Ã¿Ì¨ÒÇ±íÓĞÎ¨Ò»Í¨Ñ¶µØÖ·  Í¨Ñ¶µØÖ·´íÎó ÍË³ö  //²»Æô¶¯·¢ËÍ
			r_crc=0;
			for(i=1;i<=11;i++)
			{
				r_crc+=rs_temp[i];//1-11Ïà¼Ó
			}
			if(r_crc==rs_temp[0])//ºÍĞ£Ñé³É¹¦
			{
			  r_flag='Y';t_crc=0;
				addr=rs_temp[1];//ÒÇ±íµØÖ·
				//////////////////=============/////////////////////////
				//////////////////´ı·¢ËÍÊı¾İ´¦Àí////////////////////////
				//////////////////=============/////////////////////////
				ts_temp[0]='S';
				series_ls=tracevalue;//ÂË²¨¾ùÖµ´¦ÀíºóµÄÁ÷Á¿Öµ
				put_series_data(series_ls,xram[0]);//Êı¾İ  Ğ¡ÊıÎ»Êı  //7Î»Êı¾İ°üÀ¨Ğ¡Êıµã
				for(i=0;i<7;i++)ts_temp[1+i]=tra_temp[i];//Á÷Á¿Öµ7Î»£¨°üÀ¨Ğ¡Êıµã£© Õ¼7¸öÊı×é ×°Èë1¡ª7Êı×é   ts_temp[1]Îª×î¸ßÎ»
				ts_temp[8]='=';ts_temp[9]='P';
				series_ls=z1+z1_temp;//×ÜÀÛ¼ÆÁ¿ 
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);//P34//ÀÛ¼ÆÁ¿Ğ¡ÊıÎ»Êı
				for(i=0;i<7;i++)ts_temp[10+i]=tra_temp[i];//10-16
				ts_temp[17]='=';ts_temp[18]='B';
// 				if(fser==0)                              //¸Ä
				series_ls=totalshift[shift]+z1_temp;//°àÀÛ¼Æ
// 				else//fser=1 »»°àÊ±¿Ì
// 				{
// 					if(shift==0)
// 				  series_ls=totalshift[shift+2];
// 					else
// 					series_ls=totalshift[shift-1];//
// 					lsfser=1;//±äÁ¿Ã»ÓĞÓÃ
// 				}
				if(series_ls<0)series_ls=0; 
				put_series_data(series_ls,xram[34]);
				for(i=0;i<7;i++)ts_temp[19+i]=tra_temp[i];//19-25  °àÀÛ¼Æ
				
				ts_temp[26]='=';ts_temp[27]='V';//
				series_ls=speedvalue;//ËÙ¶ÈÖµ
				put_series_data(series_ls,xram[4]);	
				for(i=0;i<7;i++)ts_temp[28+i]=tra_temp[i];//28-34
				
				ts_temp[35]='=';ts_temp[36]='W';//ºÉÖØ
				series_ls=pressvalue;
				put_series_data(series_ls,xram[2]);	
				for(i=0;i<7;i++)ts_temp[37+i]=tra_temp[i];//37-43
				
				ts_temp[44]='=';ts_temp[45]='K';
				if(run)//ÔËĞĞĞÅºÅ  status[0]ÊÇ±¸Í×£º  status[5]¡¢status[6]ÊÇ°àÀÛ¼Æ  //ÊÂ¼şĞÅÏ¢´®¿Ú·¢ËÍ  ·¢ËÍACSII
					status[1]=0x31;//ÆôÍ£×´Ì¬
				else
					status[1]=0x30;
				if(h9)		//ÅÜÆ«»òÏÖ³¡¿ØÖÆ
					status[2]=0x31;
				else
					status[2]=0x30;
				if((h2)||(l2))//Æ¤´øÊµ¼Ê¸ºÔØ³¬³öÉè¶¨Öµ¼«ÏŞ
					status[3]=0x31;
				else
					status[3]=0x30;
				if(h5)//Æ«²î³¬³ö
					status[4]=0x31;
				else
					status[4]=0x30;
				for(i=0;i<7;i++)tra_temp[i]=status[i];
				for(i=0;i<7;i++)ts_temp[46+i]=tra_temp[i];//46-52
				
				ts_temp[53]='=';ts_temp[54]='C';
				ts_temp[55]=r_flag;//ÉÏÒ»´Î½ÓÊÜÕıÈ··¢ËÍY£¬·ñÔò·¢ËÍN
				ts_temp[56]=xram[44];//P44´ú±í±¾»úµØÖ·
// 				ts_temp[56]='1';
				for(i=0;i<4;i++)ts_temp[i+57]=' ';//·¢ËÍ4¸ö¿Õ¸ñ 57-60
				for(i=0;i<61;i++)t_crc+=ts_temp[i];
				t_crc=t_crc+0x24;//0x24 =$  
				ts_temp[61]=t_crc;
// 				ts_temp[61]='A';
				//////////////////=============/////////////////////////
				//////////////////=============/////////////////////////
// 				UCSR1B|=(1<<TXCIE1);//·¢ËÍ½áÊøÖĞ¶ÏÊ¹ÄÜ
// 				UCSR1A&=0xbf;//·¢ËÍ½áÊø±êÖ¾Î»ÇåÁã
// 				PORTJ|=0x18;//DE =1 RE =1 Ê¹ÄÜ485·¢ËÍ£¬¹Ø±Õ485½ÓÊÜ
// 				UDR1=0x24;//·¢ËÍÇ°  //Æô¶¯·¢ËÍ  $
				M485DEI=1;M485REI=1;//DE =1 RE =1 Ê¹ÄÜ485·¢ËÍ£¬¹Ø±Õ485½ÓÊÜ
				USART3->CR1|=0x0008;//Ê¹ÄÜ·¢ËÍ
				USART_ClearFlag(USART3, USART_FLAG_TC);
				USART_ITConfig(USART3, USART_IT_TC, ENABLE);return; //·¢ËÍÖĞ¶ÏÊ¹ÄÜ 
			}
			else
			{
				r_crc=0;//½ÓÊÜĞ£Ñé±äÁ¿
				r_flag='N';//½ÓÊÜĞ£ÑéÊ§°Ü	
			}
		}
	}
			
 
// 	 USART_ITConfig(USART1, USART_IT_TC, DISABLE);  
 //  USART_SendData(USART1,usar1_temp[ab] );	
//   USART_ClearFlag(USART1, USART_FLAG_TC);

// 		UCSR1A&=0xbf;////·¢ËÍ½áÊø±êÖ¾Î»ÇåÁã	
 if( USART_GetITStatus(USART3, USART_IT_TC) == SET  ) //·¢ËÍÍê³ÉÖĞ¶Ï   ·¢ËÍ¸øÁË30sµÄÊ±¼ä£¬Ê±¼äµ½£¬ÏµÍ³´¦ÓÚ½ÓÊÕ×´Ì¬
{ 
    USART_SendData(USART3,0X24);       
	if((r_flag!=0x11)&&(addr==xram[44]))//¼ìÑé³É¹¦ºó²Å·¢ËÍ£¬r_flag ="Y"
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
			case 61:USART_SendData(USART3, ts_temp[61]);sart++;break;//Ğ£ÑéºÍ
      case 62:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//´®¿Ú·¢ËÍÊ¹ÄÜ¹Ø±Õ  DE=0 RE=0  485 ¹Ø±Õ·¢ËÍ Ê¹ÄÜ½ÓÊÕ
 			default:sart=0;r_flag=0x11;USART3->CR1&=~(0x0008);USART_ITConfig(USART1, USART_IT_TC, DISABLE);M485DEI=0;M485REI=0;break;//´®¿Ú·¢ËÍÊ¹ÄÜ¹Ø±Õ  DE=0 RE=0  485 ¹Ø±Õ·¢ËÍ Ê¹ÄÜ½ÓÊÕ
		}
	}
	
  }

} 

/*********************************´®¿Ú2ÖĞ¶Ï·şÎñ³ÌĞò£¨·¢ËÍÍê³ÉÖĞ¶Ï£©********************************/
 void USART2_IRQHandler(void)   
{
  if( USART_GetITStatus(USART2, USART_IT_TC) == SET  ) //·¢ËÍÍê³ÉÖĞ¶Ï
  {
		if(xram[57]==1) //´òÓ¡Ñ¡Ôñ
		{
			switch(tx0_count)//·¢ËÍ¼ÆÊıÆ÷  //·¢ËÍ Ê±¼ä  1°àÀÛ¼Æ 2°àÀÛ¼Æ 3°àÀÛ¼Æ ×ÜÀÛ¼Æ
			{
				case 0:USART_SendData(USART2, prn_time[0]);tx0_count++;break;//´òÓ¡Ê±¼ä  Äê     
				case 1:USART_SendData(USART2, prn_time[1]);tx0_count++;break;
				case 2:USART_SendData(USART2, '-');tx0_count++;break;
				case 3:USART_SendData(USART2, prn_time[2]);tx0_count++;break;          //ÔÂ
				case 4:USART_SendData(USART2, prn_time[3]);tx0_count++;break;
				case 5:USART_SendData(USART2, '-');tx0_count++;break;	
				case 6:USART_SendData(USART2, prn_time[4]);tx0_count++;break;          //ÈÕ
				case 7:USART_SendData(USART2, prn_time[5]);tx0_count++;break;
				case 8:USART_SendData(USART2, 0x0d);tx0_count++;break; //»Ø³µ
				case 9:USART_SendData(USART2, 0x0a);tx0_count++;break; //»»ĞĞ
				case 10:USART_SendData(USART2, prn_time[6]);tx0_count++;break;         //Ê±
				case 11:USART_SendData(USART2, prn_time[7]);tx0_count++;break;
				case 12:USART_SendData(USART2, 0x3a);tx0_count++;break;  //:
				case 13:USART_SendData(USART2, prn_time[8]);tx0_count++;break;         //·Ö
				case 14:USART_SendData(USART2, prn_time[9]);tx0_count++;break;
				case 15:USART_SendData(USART2, 0x3a);tx0_count++;break; //:
				case 16:USART_SendData(USART2, prn_time[10]);tx0_count++;break;        //Ãë
				case 17:USART_SendData(USART2, prn_time[11]);tx0_count++;break;
				case 18:USART_SendData(USART2, 0x0d);tx0_count++;break;//»Ø³µ Enter
				case 19:USART_SendData(USART2, 0X0a);tx0_count++;break;//»»ĞĞ
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
				case 42:USART_SendData(USART2, prnbuf1[7]);tx0_count++;break;//prnbuf1  1°à×éÀÛ¼ÆÁ¿
				case 43:USART_SendData(USART2, prnbuf1[6]);tx0_count++;break;
				case 44:USART_SendData(USART2, prnbuf1[5]);tx0_count++;break;
				case 45:USART_SendData(USART2, prnbuf1[4]);tx0_count++;break;
				case 46:USART_SendData(USART2, prnbuf1[3]);tx0_count++;break;
				case 47:USART_SendData(USART2, prnbuf1[2]);tx0_count++;break;
				case 48:USART_SendData(USART2, prnbuf1[1]);tx0_count++;break;
				case 49:USART_SendData(USART2, prnbuf1[0]);tx0_count++;break;
				case 50:USART_SendData(USART2, 0x74);tx0_count++;break;//t
				case 51:USART_SendData(USART2, 0x0d);tx0_count++;break;//»Ø³µ»»ĞĞ
				case 52:USART_SendData(USART2, 0x0a);tx0_count++;break;
				case 53:USART_SendData(USART2, 'S');tx0_count++;break;//SHIFT:1
				case 54:USART_SendData(USART2, 'H');tx0_count++;break;
				case 55:USART_SendData(USART2, 'I');tx0_count++;break;
				case 56:USART_SendData(USART2, 'F');tx0_count++;break;
				case 57:USART_SendData(USART2, 'T');tx0_count++;break;
				case 58:USART_SendData(USART2, ':');tx0_count++;break;
				case 59:USART_SendData(USART2, 0x31);tx0_count++;break;//1
				case 60:USART_SendData(USART2, 0x0d);tx0_count++;break;//»Ø³µ
				case 61:USART_SendData(USART2, 0x0a);tx0_count++;break;//»»ĞĞ
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
				case 84:USART_SendData(USART2, prnbuf2[7]);tx0_count++;break;//prnbuf2  °à×é2
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
				case 126:USART_SendData(USART2, prnbuf3[7]);tx0_count++;break;//prnbuf3  °à×é3
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
				case 168:USART_SendData(USART2, prnbuf4[7]);tx0_count++;break;//prnbuf4 ×ÜµÄÀÛ¼ÆÁ¿ 
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
				case 193:USART_SendData(USART2, 0x0d);tx0_count++;break;//»Ø³µ Enter	
				case 194:USART_SendData(USART2, 0x0a);tx0_count++;break;//»»ĞĞ
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

void EXTI9_5_IRQHandler(void) //±ê¶¨È¦Âö³åÊı
{
 if( EXTI_GetITStatus( EXTI_Line5)!=RESET)
 { 
		/////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ±¸·İ////////   
		e5_val=SysTick->VAL;//²»Îª0 Ôò 16Î» Îª0 Îª0 16Î»ÇåÁã
		if(e5_val==0) SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;//¹Ø±Õ¼ÆÊıÆ÷1
		e5_ctrl=SysTick->CTRL;  
	  EXTI_ClearITPendingBit(EXTI_Line5);  //Çå³ıLINE15ÉÏµÄÖĞ¶Ï±êÖ¾Î»
//	EXTI_Set_Int(0);//¹ØÍâ²¿ÖĞ¶ÏÍ£Ö¹¼ÇÈ¦Êı   
// 	  tare_s=0;
 if(calflg==1)
 {
   time_flag=1;
   EXTI_Set_Int(0);//¹ØÍâ²¿ÖĞ¶ÏÍ£Ö¹¼ÇÈ¦Êı
  if(tare_cycle_flag==1)
  {
	  if(cycle_flag==1)//µÚÒ»´Î½øÈëÖĞ¶Ï¿ª¼ÆÊıÆ÷1 ¼ÆÂö³åÊı
		{ 
			TIM_SetCounter(TIM1, 0);//¼ÆÊıÆ÷ÇåÁã
			TIM_Cmd(TIM1, ENABLE);  //Ê¹ÄÜTIM1 //TIM1¶¨Ê±Æ÷Ê¹ÄÜ	 ±ê¶¨Ê±È¦Âö³åÊı²Å¿ª	
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
        EXTI_Set_Int(0);//¹ØÍâ²¿ÖĞ¶ÏÍ£Ö¹¼ÇÈ¦Êı
		    cycle_flag=1;
	      TIM_Cmd(TIM1, DISABLE);//¹Ø¶¨Ê±Æ÷
      }
  }
  else//tare_cycle_flag=0
  {
// 	  tareline=1;   //½øÈëÖĞ¶ÏºóÅĞ¶ÏÊÇ·ñ´ÓÁãµã¿ªÊ¼³ÆÖØ
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
      EXTI_Set_Int(0);//¹ØÍâ²¿ÖĞ¶ÏÍ£Ö¹¼ÇÈ¦Êı
	    tare_flag=1;
			TIM_Cmd(TIM1, DISABLE);//¹Ø¶¨Ê±Æ÷
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
//       EXTI_Set_Int(0);//¹ØÍâ²¿ÖĞ¶ÏÍ£Ö¹¼ÇÈ¦Êı
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
 
	/////////////SysTickÑÓÊ±¼Ä´æÆ÷Êı¾İ»¹Ô­////////
		 SysTick->LOAD=e5_val;
		 SysTick->VAL=0x00;
		 SysTick->CTRL=e5_ctrl;	
 }	   
}




#include "delay.h"
#include "Data_Processing.h"
#include "math.h"
extern int flage_display;//0为不显示

//	float  setpoint;       //设定值
//	float Kp;              //比例系数
//	float Kd;       			 //积分系数
//	float Ki;    					 //微分系数
//	float lasterror;       //前一拍偏差   
//	float preerror;        //前两拍偏差
//	float result; 			   //输出值
void PID_int(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //目标值
	vPID->Kd=0.01;       //微分系数
	vPID->Ki=0.2;      //积分系数
	vPID->Kp=0.8;       //比例系数
	vPID->Kp2=2;       //比例系数
	vPID->lasterror=0.0;//前一拍偏差 
	vPID->preerror=0.0; //前两拍偏差
	vPID->result=0.0;   //输出值
}
void PID_power_cap_init(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //目标值
	vPID->Kd=0.001;       //微分系数
	vPID->Ki=0.002;      //积分系数
	vPID->Kp=0.01;       //比例系数
	vPID->Kp2=0.02;       //比例系数
	vPID->lasterror=0.0;//前一拍偏差 
	vPID->preerror=0.0; //前两拍偏差
	vPID->result=0.0;   //输出值
}
void PID_power_out_init(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //目标值
	vPID->Kd=0.000;      //微分系数
	vPID->Ki=0.015;      //积分系数
	vPID->Kp=0.01;       //比例系数
	vPID->Kp2=0.01;      //比例系数
	vPID->lasterror=0.0;//前一拍偏差 
	vPID->preerror=0.0; //前两拍偏差
	vPID->result=20;   //输出值
}

void PIDRegulation(volatile PID *vPID, float processValue)//processValue  当前值
{
  float thisError; //偏差
  float increment; //增量
  float pError,dError,iError;
  if(vPID->setpoint<0.160)
	{
	
	}
	else if(vPID->setpoint>=0.160) 
	{
		thisError=(vPID->setpoint)-processValue; //当前误差等于设定值减去当前值
		//thisError=2-processValue; //当前误差等于设定值减去当前值
		//计算公式中除系数外的三个 乘数
		pError=thisError-vPID->lasterror;//两次偏差差值err(k)-err(k-1)
		iError=thisError;
		dError=thisError-2*(vPID->lasterror)+vPID->preerror;
		if((thisError/vPID->setpoint)<0.5)
		{
			increment=vPID->Kp*pError+vPID->Ki*iError+vPID->Kd*dError;   //增量计算
		}
		else if(thisError/vPID->setpoint>=0.5)
		{
			increment=vPID->Kp2*pError+vPID->Ki*iError;  //增量计算
		}
		

		vPID->preerror=vPID->lasterror;  //存放偏差用于下次运算
		vPID->lasterror=thisError;
		
		vPID->result+=increment;//结果当然是上次结果 加上本次增量
	}
	

}



uint16_t Average_Value(uint16_t* date,int Length)//key2
{
	uint16_t AVG=0;
	for(int i=0;i<Length;i++)
	{	
		AVG = date[i]+AVG;
		
	}
	AVG = AVG/Length ;
	
	return AVG;
}
double Standard_Deviation(uint16_t* date,int Length)//方差
{
	uint16_t date_copy[Length];
	uint16_t avg;
	int difference_value[Length];//和平均值的差值
	int difference_value_sum_2=0;//差值^2求和
	double var=0;//方差
	for(int i=0;i<Length;i++)//拷贝数据
	{
		date_copy[i]=date[i];
	
	}
	avg=Average_Value(date_copy,Length);//求数据平均值

	for(int i=0;i<Length;i++)
	{
		difference_value[i]=date_copy[i]-avg;//求和平均值的差值
	}
	for(int i=0;i<Length;i++)//求数据的方差
	{
		difference_value_sum_2=(difference_value[i]*difference_value[i])+difference_value_sum_2;//差值求和
	}
	var=sqrt(difference_value_sum_2/(Length-1));
	//var=sqrt(100);
	return var;
}
double AGV_Remove_the_Ross_Error(uint16_t* date,int Length)//去除粗大误差，莱以特准则
{
	uint16_t date_copy[Length];
	uint16_t data_marker[Length];//0为有效数据，1为无效数据
	uint16_t avg;
	int data_marker_num=0;//data_marker的数量
	double var;//方差
	int difference_value_abs[Length];//和平均值的差值
	for(int i=0;i<Length;i++)//拷贝数据
	{
		date_copy[i]=date[i];
	
	}
	
	avg=Average_Value(date_copy,Length);//求数据平均值
	for(int i=0;i<Length;i++)
	{
 		difference_value_abs[i]= abs(date_copy[i]-avg);//求和平均值的差值的绝对值
	}

	var=Standard_Deviation(date_copy,Length);//方差
	for(int i=0;i<Length;i++)//莱以特准则
	{
		if(difference_value_abs[i]>(var*3))
		{
			data_marker[i]=1;//无效数据
		}
		else
		{
			data_marker[i]=0;//有效数据
			data_marker_num=data_marker_num+1;
		}
	
	}
	avg=0;//清零
	for(int i=0;i<Length;i++)//有效数据求平均值
	{
		if(data_marker[i]==0)
		{
			avg=avg+date_copy[i];
		}
	}
	avg=avg/data_marker_num;
	return avg;

		
	
}

float kalmanFilter_A(float inData,float*prevData) 
{
/*       
        Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
        R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
				P：初始值没关系
	      kGain：初始值0
*/
  
	static float p=10, q=0.01, r=0.005, kGain=0;
	p = p+q; 
	kGain = p/(p+r);

	inData = *prevData+(kGain*(inData-*prevData)); 
	p = (1-kGain)*p;

	*prevData = inData;

	return inData; 
}



















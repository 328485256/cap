#include "delay.h"
#include "Data_Processing.h"
#include "math.h"
extern int flage_display;//0Ϊ����ʾ

//	float  setpoint;       //�趨ֵ
//	float Kp;              //����ϵ��
//	float Kd;       			 //����ϵ��
//	float Ki;    					 //΢��ϵ��
//	float lasterror;       //ǰһ��ƫ��   
//	float preerror;        //ǰ����ƫ��
//	float result; 			   //���ֵ
void PID_int(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //Ŀ��ֵ
	vPID->Kd=0.01;       //΢��ϵ��
	vPID->Ki=0.2;      //����ϵ��
	vPID->Kp=0.8;       //����ϵ��
	vPID->Kp2=2;       //����ϵ��
	vPID->lasterror=0.0;//ǰһ��ƫ�� 
	vPID->preerror=0.0; //ǰ����ƫ��
	vPID->result=0.0;   //���ֵ
}
void PID_power_cap_init(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //Ŀ��ֵ
	vPID->Kd=0.001;       //΢��ϵ��
	vPID->Ki=0.002;      //����ϵ��
	vPID->Kp=0.01;       //����ϵ��
	vPID->Kp2=0.02;       //����ϵ��
	vPID->lasterror=0.0;//ǰһ��ƫ�� 
	vPID->preerror=0.0; //ǰ����ƫ��
	vPID->result=0.0;   //���ֵ
}
void PID_power_out_init(PID *vPID)
{
	vPID->temp=0;
	vPID->setpoint=0.0; //Ŀ��ֵ
	vPID->Kd=0.000;      //΢��ϵ��
	vPID->Ki=0.015;      //����ϵ��
	vPID->Kp=0.01;       //����ϵ��
	vPID->Kp2=0.01;      //����ϵ��
	vPID->lasterror=0.0;//ǰһ��ƫ�� 
	vPID->preerror=0.0; //ǰ����ƫ��
	vPID->result=20;   //���ֵ
}

void PIDRegulation(volatile PID *vPID, float processValue)//processValue  ��ǰֵ
{
  float thisError; //ƫ��
  float increment; //����
  float pError,dError,iError;
  if(vPID->setpoint<0.160)
	{
	
	}
	else if(vPID->setpoint>=0.160) 
	{
		thisError=(vPID->setpoint)-processValue; //��ǰ�������趨ֵ��ȥ��ǰֵ
		//thisError=2-processValue; //��ǰ�������趨ֵ��ȥ��ǰֵ
		//���㹫ʽ�г�ϵ��������� ����
		pError=thisError-vPID->lasterror;//����ƫ���ֵerr(k)-err(k-1)
		iError=thisError;
		dError=thisError-2*(vPID->lasterror)+vPID->preerror;
		if((thisError/vPID->setpoint)<0.5)
		{
			increment=vPID->Kp*pError+vPID->Ki*iError+vPID->Kd*dError;   //��������
		}
		else if(thisError/vPID->setpoint>=0.5)
		{
			increment=vPID->Kp2*pError+vPID->Ki*iError;  //��������
		}
		

		vPID->preerror=vPID->lasterror;  //���ƫ�������´�����
		vPID->lasterror=thisError;
		
		vPID->result+=increment;//�����Ȼ���ϴν�� ���ϱ�������
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
double Standard_Deviation(uint16_t* date,int Length)//����
{
	uint16_t date_copy[Length];
	uint16_t avg;
	int difference_value[Length];//��ƽ��ֵ�Ĳ�ֵ
	int difference_value_sum_2=0;//��ֵ^2���
	double var=0;//����
	for(int i=0;i<Length;i++)//��������
	{
		date_copy[i]=date[i];
	
	}
	avg=Average_Value(date_copy,Length);//������ƽ��ֵ

	for(int i=0;i<Length;i++)
	{
		difference_value[i]=date_copy[i]-avg;//���ƽ��ֵ�Ĳ�ֵ
	}
	for(int i=0;i<Length;i++)//�����ݵķ���
	{
		difference_value_sum_2=(difference_value[i]*difference_value[i])+difference_value_sum_2;//��ֵ���
	}
	var=sqrt(difference_value_sum_2/(Length-1));
	//var=sqrt(100);
	return var;
}
double AGV_Remove_the_Ross_Error(uint16_t* date,int Length)//ȥ���ִ���������׼��
{
	uint16_t date_copy[Length];
	uint16_t data_marker[Length];//0Ϊ��Ч���ݣ�1Ϊ��Ч����
	uint16_t avg;
	int data_marker_num=0;//data_marker������
	double var;//����
	int difference_value_abs[Length];//��ƽ��ֵ�Ĳ�ֵ
	for(int i=0;i<Length;i++)//��������
	{
		date_copy[i]=date[i];
	
	}
	
	avg=Average_Value(date_copy,Length);//������ƽ��ֵ
	for(int i=0;i<Length;i++)
	{
 		difference_value_abs[i]= abs(date_copy[i]-avg);//���ƽ��ֵ�Ĳ�ֵ�ľ���ֵ
	}

	var=Standard_Deviation(date_copy,Length);//����
	for(int i=0;i<Length;i++)//������׼��
	{
		if(difference_value_abs[i]>(var*3))
		{
			data_marker[i]=1;//��Ч����
		}
		else
		{
			data_marker[i]=0;//��Ч����
			data_marker_num=data_marker_num+1;
		}
	
	}
	avg=0;//����
	for(int i=0;i<Length;i++)//��Ч������ƽ��ֵ
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
        Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
        R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��
				P����ʼֵû��ϵ
	      kGain����ʼֵ0
*/
  
	static float p=10, q=0.01, r=0.005, kGain=0;
	p = p+q; 
	kGain = p/(p+r);

	inData = *prevData+(kGain*(inData-*prevData)); 
	p = (1-kGain)*p;

	*prevData = inData;

	return inData; 
}



















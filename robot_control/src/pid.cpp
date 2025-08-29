#include "robot_control/pid.hpp"

/**
  * @brief 绝对式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅.
  * @note   .
  * @retval none.
  */
void pid_init_absolute(PID_AbsoluteType* PID,float kp, float ki, float kd, float I_limit, float Out_Limit)
{
	PID->kp		= kp;
	PID->ki		= ki;
	PID->kd		= kd;
	PID->I_Lim  = I_limit;
	PID->errNow = 0;
	PID->errP   = 0;
	PID->errI   = 0;
	PID->errD   = 0;
	PID->errOld = 0;
	PID->ctrOut = 0;
    PID->Out_Limit = Out_Limit;
}

/**
  * @brief  普通绝对式PID.
  * @param  目标值，实际值，PID结构体地址.
  * @note   .
  * @retval 需要输出的值.
  */
float pid_absolute(float Target,float Current,PID_AbsoluteType* PID)
{
	PID->errNow = Target - Current;

	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow;//误差积分，用于ki控制

	if(PID->I_Lim != 0)	   
	{
		if(PID->errI >  PID->I_Lim)  
			PID->errI =  PID->I_Lim;
		else if(PID->errI <  -PID->I_Lim)  
			PID->errI = -PID->I_Lim;
	}	
 
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制

	PID->errOld = PID->errNow;	//保存现在的误差
 
	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出
	
	if(PID->ctrOut > PID->Out_Limit)
		PID->ctrOut = PID->Out_Limit;
	else if(PID->ctrOut < -PID->Out_Limit)
		PID->ctrOut = -PID->Out_Limit;
	return PID->ctrOut;
}

/**
  * @brief  三段绝对式PID初始化
  * @param  kp及差值err分段，ki，kd，限幅
  * @note   分段赋值
  * @retval 无
*/
void pid_init_absolute_threesection(PID_AbsoluteType_ThreeSection *PID,float *kp,float *err,float ki,float kd,float I_Limit,float Out_Limit)
{
	PID->tempkp[0]  = kp[0];
	PID->tempkp[1]  = kp[1];
	PID->tempkp[2]  = kp[2];
	PID->tempkp[3]  = kp[3];
	PID->err[0]     =  err[0];
	PID->err[1]     =  err[1];		
	PID->err[2]     =  err[2];	
	PID->err[3]     =  err[3];
	PID->ki		    = ki;
	PID->kd		    = kd;
	PID->I_Lim      = I_Limit;
	PID->Out_Limit  = Out_Limit;
}

/**
  * @brief  三段绝对式PID
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
*/
float pid_absolute_threesection_update(float Target, float Current,PID_AbsoluteType_ThreeSection *PID)
{
	PID->errNow = Target - Current;
	
	if(PID->errNow > -PID->err[0] && PID->errNow < PID->err[0])
		PID->kp = PID ->errI = 0;
	else if(PID->errNow > -PID->err[1] && PID->errNow < PID->err[1])
		PID->kp = PID ->tempkp[0];
	else if(PID->errNow > -PID->err[2] && PID->errNow < PID->err[2])
		PID->kp = PID->tempkp[1];
	else if(PID->errNow > -PID->err[3] && PID->errNow < PID->err[3])
		PID->kp = PID->tempkp[2];
	else 
		PID->kp = PID->tempkp[3];
  
	PID->errP = PID->errNow;  //读取现在的误差，用于kp控制
	
	PID->errI += PID->errNow; //误差积分，用于ki控制
	
	if(PID->I_Lim!=0)	   //积分上限和下限
	{
		if(PID->errI>PID->I_Lim)  
			PID->errI=PID->I_Lim;
		else if(PID->errI<-PID->I_Lim)  
			PID->errI=-PID->I_Lim;
	}
	PID->errD = PID->errNow - PID->errOld;//误差微分，用于kd控制
	PID->errOld = PID->errNow;	//保存现在的误差

	PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//计算绝对式PID输出

	if (PID->ctrOut >  PID->Out_Limit)
      PID->ctrOut =  PID->Out_Limit;
  if (PID->ctrOut < -PID->Out_Limit)
      PID->ctrOut = -PID->Out_Limit; 

	return PID->ctrOut;
}

/**
  * @brief  位置式积分限幅+积分分离PID
  * @param  PID结构体地址，P,I,D,输出限幅，积分限幅，积分分离
  * @note   .
  * @retval none.
  */
void pid_init_antiintegral(PID_AntiIntegralType *PID, float kp, float ki, float kd, float Out_Limit,float I_Limit,float I_Band)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->Out_Limit = Out_Limit;
    PID->I_Limit = I_Limit;
	PID->I_Band = I_Band;
}

/**
  * @brief  抗积分饱和PID（I限幅）
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_antiintegral_update(float Target,float Current,PID_AntiIntegralType* PID)
{
	  PID->errNow = Target - Current;
	  PID->errNow_fabs=(PID->errNow>0)?(PID->errNow):(-PID->errNow);

    if(PID->I_Band!=0)
	{
	    if(PID->errNow_fabs<PID->I_Band)
            PID->errI = PID->errI + PID->ki* PID->errNow;	//积分计算，ki控制
        else
            PID->errI = 0;
    }
    else
    {
        PID->errI = PID->errI + PID->ki* PID->errNow;
    }
		
    if(PID->I_Limit!=0)
    {
        if(PID->errI > PID->I_Limit)	
            PID->errI = PID->I_Limit;
        if(PID->errI < -PID->I_Limit)
            PID->errI = -PID->I_Limit;
    }

	PID->errP = PID->kp *  PID->errNow;//比例计算，kp控制 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //微分计算，kd控制
	  
    PID->ctrOut=PID->errP+PID->errI+PID->errD;
    
    if (PID->ctrOut > PID->Out_Limit)
        PID->ctrOut = PID->Out_Limit;
    if (PID->ctrOut < -PID->Out_Limit)
        PID->ctrOut = -PID->Out_Limit; 

    PID->errOld = PID->errNow; //保存现在的误差
    return PID->ctrOut;
}

/**
  * @brief  分段抗积分饱和PID（I限幅）
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_antiintegral_update_section(float Target,float Current,PID_AntiIntegralType* PID,float* kpid)
{
    PID->kp = kpid[0];
    PID->ki = kpid[1];
    PID->kd = kpid[2];
	
    PID->errNow = Target - Current;
    PID->errNow_fabs=(PID->errNow>0)?(PID->errNow):(-PID->errNow);

    if(PID->I_Band!=0)
    {
        if(PID->errNow_fabs<PID->I_Band)
            PID->errI = PID->errI + PID->ki* PID->errNow;	//积分计算，ki控制
        else
            PID->errI = 0;
    }
    else
    {
        PID->errI = PID->errI + PID->ki* PID->errNow;
    }

    if(PID->I_Limit!=0)
    {
        if(PID->errI > PID->I_Limit)	
            PID->errI = PID->I_Limit;
        if(PID->errI < -PID->I_Limit)
            PID->errI = -PID->I_Limit;
    }

	PID->errP = PID->kp *  PID->errNow;//比例计算，kp控制 
    PID->errD = PID->kd * (PID->errNow - PID->errOld); //微分计算，kd控制
	  
    PID->ctrOut=PID->errP+PID->errI+PID->errD;
    
    if (PID->ctrOut > PID->Out_Limit)
        PID->ctrOut = PID->Out_Limit;
    if (PID->ctrOut < -PID->Out_Limit)
        PID->ctrOut = -PID->Out_Limit; 

    PID->errOld = PID->errNow; //保存现在的误差
    return PID->ctrOut;
}

/*前馈
采样周期：T = 0.001s;
转动惯量： J = 1;
摩擦系数：f = 1;
电机传递函数：G(s) = K/(sTJ+1)
G(s)*Gf(s) = 1;
角速度/力矩：G(s) = 1/(s+1);   
前馈环节：Gf(s) = s + 1;         （前馈传递函数）
输出为 out = (in - last_in)/T + in*k2;
*/	
double forwardfeed(ForwardFeed* ffd,double in)      //in为误差输入量，跟PID的输入量一致
{
	ffd->in = in;
	ffd->out = 1.2*(ffd->in - ffd->last_in)*ffd->T1+ffd->in*ffd->T2;
	ffd->last_in = ffd->in;
	return ffd->out;
}

void ffdInit(ForwardFeed* ffd,double T1,double T2)
{
	ffd->in = 0;
	ffd->last_in = 0;
	ffd->out = 0;
    ffd->T1 = T1;
	ffd->T2 = T2;
}

/**
  * @brief  增量式PID初始化.
  * @param  PID结构体地址，P,I,D,积分限幅，增量限幅.
  * @note   .
  * @retval none.
  */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim,float OutLim) //PID初始化系数
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
	PID->OutLim = OutLim;
}

/**
  * @brief  普通的增量式PID（可以进行增量限幅）
  * @param  目标值，反馈值，PID结构体地址.
  * @note   返回的并非是增量，而直接是需要输出的值.
  * @retval 需要输出的量.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //误差量：目标-目前（相邻两次测量值取差分）

    dErrP = PID->errNow - PID->errOld1;					   //计算比例分量----微分：现误差-上一个误差
    dErrI = PID->errNow;								   //计算积分分量——————现误差
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //计算微分分量——————现误差-2*一阶误差微分+二阶误差微分

//		if (PID->errNow < 10 && PID->errNow > -10)
//			dErrI = 0;
		
    /*增量式PID计算*/
    PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID合成输出量

    PID->errOld2 = PID->errOld1; //二阶误差微分
    PID->errOld1 = PID->errNow;  //一阶误差微分


    if (PID->dCtrOut < -PID->IncLim)
        PID->dCtrOut = -PID->IncLim;
    else if (PID->dCtrOut > PID->IncLim)
        PID->dCtrOut = PID->IncLim;

    PID->ctrOut += PID->dCtrOut;
    if(PID->ctrOut>PID->OutLim)
        PID->ctrOut=PID->OutLim;
    if(PID->ctrOut<-PID->OutLim)
        PID->ctrOut=-PID->OutLim;
    return PID->ctrOut;
}


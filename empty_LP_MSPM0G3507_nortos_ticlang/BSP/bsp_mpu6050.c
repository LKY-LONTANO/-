#include "bsp_mpu6050.h"
#include "bsp_siic.h"

Imu_t mpu6050 = { 0 };
Imu_t RegOri_mpu6050 = { 0 };

//use demo
uint8_t mpu6050_init(void)
{
	pIICInterface_t iic = &User_sIICDev;
	
	IIC_Status_t check_state = IIC_OK;//检查初始化步骤是否出错
	
	uint8_t iobuf = 0;
	
	//访问mpu6050设备是否存在
	check_state += iic->read_reg(MPU6050_DEV<<1,MPU6050_RA_WHO_AM_I,&iobuf,1,500);
	if( iobuf!=MPU6050_DEV ) return 1;
	
	//复位mpu6050
	iobuf = 1<<7;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_PWR_MGMT_1,&iobuf,1,500);
	
	//延迟200ms等待mpu6050复位成功
	iic->delay_ms(200);
	
	//唤醒mpu6050,并设置X轴作为陀螺仪的时钟源(手册推荐X、Y、Z轴均可)
	iobuf = 1<<0; //时钟源[bit2~bit0]
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_PWR_MGMT_1,&iobuf,1,500);
	
	//设置加速度于陀螺仪都作用(关闭低功耗模式)
	iobuf = 0;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_PWR_MGMT_2,&iobuf,1,500);
	
	//设置gyro量程,±2000dps.  [bit4~bit3] 0:250 1:500 2:1000 3:2000
	iobuf = 3<<3;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_GYRO_CONFIG,&iobuf,1,500);
	
	//设置accel量程,±2g. [bit4~bit3] 0:2g 1:4g 2:8g 3:16g
	iobuf = 0<<4;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_ACCEL_CONFIG,&iobuf,1,500);
	
	//设置滤波带宽DLPF为2,数据大约有3ms的滞后.此时陀螺仪数据的输出频率为1Khz(DLPF=0或7时,输出频率为8Khz,其他值时1kHz)
	iobuf = 2<<0;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_CONFIG,&iobuf,1,500);
	
	//设置陀螺仪数据采样率,设置为200Hz.保证读取的数据为最新数据. 采样率 = 陀螺仪输出频率 / ( 1+SMPLRT_DIV )
	iobuf = 20;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_SMPLRT_DIV,&iobuf,1,500);
	
	//关闭mpu6050所有中断输出
	iobuf = 0;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_INT_ENABLE,&iobuf,1,500);
	
	//关闭所有fifo
	iobuf = 0;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_FIFO_EN,&iobuf,1,500);
	
	//关闭iic主模式,关闭FIFO使能,启用IIC接口（I2C_MST_EN=0）
	iobuf = 0;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_USER_CTRL,&iobuf,1,500);
	
	//启用bypass模式,操作其他iic设备不需要经过mpu6050（需要bypassEN和I2C_MST_EN=0）
	iobuf = 1<<1;
	check_state += iic->write_reg(MPU6050_DEV<<1,MPU6050_RA_INT_PIN_CFG,&iobuf,1,500);
	
	if( check_state!=IIC_OK ) return 1;
	
	return 0;
}

//快速计算 1/sqrt(x)
static float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 

static const float Kp = 0.8f;   // 比例系数
static const float Ki = 0.0001f; // 积分系数

//更新欧拉角
static void mpu6050_UpdateAttitude(Imu_t* imudata)
{
    static float eInt[3] = {0}; // 初始化累计误差
    static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始化四元数
    float norm;
    float vx, vy, vz;
    float ex, ey, ez; // 误差变量
	float q0_t,q1_t,q2_t,q3_t;
	
	float halftime = 0.01f * 0.5f; // 5ms的采样周期
	
	// 辅助变量（简化变量名）
	float gx = imudata->gyro.x;  // 绕X轴旋转的角速度(rad/s)
	float gy = imudata->gyro.y;  // 绕Y轴旋转的角速度(rad/s)
	float gz = imudata->gyro.z;  // 绕Z轴旋转的角速度(rad/s)
	float ax = imudata->accel.x; // 沿X轴移动的加速度(m·s^-2)
	float ay = imudata->accel.y; // 沿Y轴移动的加速度(m·s^-2)
	float az = imudata->accel.z; // 沿Z轴移动的加速度(m·s^-2)

    // 辅助变量（避免冗长的计算表达式）
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	
    // 提取等效旋转矩阵中的重力分量 
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
	vz = 1-2*(q1q1+q2q2);
	
	//加速度计归一化
	norm = Q_rsqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 计算误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 累计误差
    eInt[0] += ex;
    eInt[1] += ey;
    eInt[2] += ez;

    // PI控制
    gx += Kp * ex + Ki * eInt[0];
    gy += Kp * ey + Ki * eInt[1];
    gz += Kp * ez + Ki * eInt[2];

    // 更新四元数
    q0_t =  (-q1 * gx - q2 * gy - q3 * gz) * halftime;
    q1_t =  ( q0 * gx + q2 * gz - q3 * gy) * halftime;
    q2_t =  ( q0 * gy - q1 * gz + q3 * gx) * halftime;
    q3_t =  ( q0 * gz + q1 * gy - q2 * gx) * halftime;
	
	q0+=q0_t;
	q1+=q1_t;
	q2+=q2_t;
	q3+=q3_t;
	
    // 归一化（四元数）
    norm = Q_rsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

	// 四元数转换成欧拉角
	imudata  -> roll   = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.295779513f;
	imudata -> pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.295779513f;
	imudata -> yaw   = atan2(2 *(q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.295779513f;
}

void imu_solving(void)
{
	//IIC驱动接口
	pIICInterface_t siic = &User_sIICDev;

	//读取mpu6050原始数据,操作耗时2ms
	uint8_t tmpbuf[14]={0};
	siic -> read_reg(MPU6050_DEV<<1,MPU6050_RA_ACCEL_XOUT_H,tmpbuf,14,100);

	//原始数据
	mpu6050.accel.x = (short)(tmpbuf[0]<<8|tmpbuf[1]);
	mpu6050.accel.y = (short)(tmpbuf[2]<<8|tmpbuf[3]);
	mpu6050.accel.z = (short)(tmpbuf[4]<<8|tmpbuf[5]);
	mpu6050.gyro.x = (short)(tmpbuf[8]<<8|tmpbuf[9]);
	mpu6050.gyro.y = (short)(tmpbuf[10]<<8|tmpbuf[11]);
	mpu6050.gyro.z = (short)(tmpbuf[12]<<8|tmpbuf[13]);

	//原始数据
	RegOri_mpu6050.gyro.x = mpu6050.gyro.x;
	RegOri_mpu6050.gyro.z = mpu6050.gyro.z;

	//单位换算为 rad/s
	// mpu6050.gyro.x*=0.0152587890625f;//°/s
	// mpu6050.gyro.y*=0.0152587890625f;
	// mpu6050.gyro.z*=0.0152587890625f;

	mpu6050.gyro.x*=0.06103515625f;
	mpu6050.gyro.y*=0.06103515625f;
	mpu6050.gyro.z*=0.06103515625f;

	mpu6050.gyro.x*=0.01745329252f;//rad/s
	mpu6050.gyro.y*=0.01745329252f;
	mpu6050.gyro.z*=0.01745329252f;

	//单位换算为 m/s^2
	mpu6050.accel.x*=0.00059814453125f;
	mpu6050.accel.y*=0.00059814453125f;
	mpu6050.accel.z*=0.00059814453125f;

	mpu6050_UpdateAttitude(&mpu6050);
}


#include "KF.h"
void imu_solving_KF(void)
{
	#define PI 3.14159265f	

	//IIC驱动接口
	pIICInterface_t siic = &User_sIICDev;

	//读取mpu6050原始数据,操作耗时2ms
	uint8_t tmpbuf[14]={0};
	siic -> read_reg(MPU6050_DEV<<1,MPU6050_RA_ACCEL_XOUT_H,tmpbuf,14,100);

	//原始数据
	mpu6050.accel.x = (short)(tmpbuf[0]<<8|tmpbuf[1]);
	mpu6050.accel.y = (short)(tmpbuf[2]<<8|tmpbuf[3]);
	mpu6050.accel.z = (short)(tmpbuf[4]<<8|tmpbuf[5]);
	mpu6050.gyro.x = (short)(tmpbuf[8]<<8|tmpbuf[9]);
	mpu6050.gyro.y = (short)(tmpbuf[10]<<8|tmpbuf[11]);
	mpu6050.gyro.z = (short)(tmpbuf[12]<<8|tmpbuf[13]);

	//原始数据
	RegOri_mpu6050.gyro.x = mpu6050.gyro.x;
	RegOri_mpu6050.gyro.z = mpu6050.gyro.z;

	float Accel_Angle_x=atan2(mpu6050.accel.y,mpu6050.accel.z)*180/PI;     //计算倾角，转换单位为度	
	float Accel_Angle_y=atan2(mpu6050.accel.x,mpu6050.accel.z)*180/PI;     //计算倾角，转换单位为度
	float accel_x=mpu6050.accel.x/1671.84f;
	float accel_y=mpu6050.accel.y/1671.84f;
	float accel_z=mpu6050.accel.z/1671.84f;
	float gyro_x=mpu6050.gyro.x/16.4f;                              //陀螺仪量程转换
	float gyro_y=mpu6050.gyro.y/16.4f;                              //陀螺仪量程转换	

	mpu6050.roll= -KF_X(accel_y,accel_z,-gyro_x);//卡尔曼滤波
	mpu6050.pitch = KF_Y(accel_x,accel_z,gyro_y);

}


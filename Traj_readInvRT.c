/*****************************************************************************
*
* FILE NAME:    Trajectory.c
*
* DESCRIPTION:  サーボ指令生成処理
*
*****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <rt.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "ProcTrajRT.h"
#include "DataConst.h"

#include "CommonVariable.h" // 共通パラメータファイル

#ifndef PI
#define PI     ((double)3.14159265358979323846)
#endif

#ifndef RAD2DEG
#define RAD2DEG ( (double)180.0 / PI )
#endif

#ifndef DEG2RAD
#define DEG2RAD ( PI / (double)180.0)
#endif

extern RTHANDLE		hMailTraj;
extern int	iArgc;
extern char	**szArgv;

int	count1000 = 0;
long	sec = 0;

#ifndef PI
#define PI     ((double)3.14159265358979323846)
#endif

//#define	LOG_ANGLE

#ifndef RAD2DEG
#define RAD2DEG ( (double)180.0 / PI )
#endif

#ifndef DEG2RAD
#define DEG2RAD ( PI / (double)180.0)
#endif

#define	EPS	1.e-8

//汎用グローバル変数
#define	NUMGLVAL	1024
extern int	glint[NUMGLVAL];
extern double	gldouble[NUMGLVAL];

//各関節の最大速度 [deg/sec]
double EnpArc_JntMaxSpd[] = { 225,	 225,	 257,	 279,	 275,	 360 };

#define	SAMPLING_INTERVAL	((double)0.001)

//C3リンク長[m]
double	EnpArc_dof6c3_a[] = { 0.100,	0.250,	0.000,	 0.000,	0.000,	 0.000 };
double	EnpArc_dof6c3_d[] = { 0.320,	0.000,	0.000,	-0.250,	0.000,	-0.065 };

//C3可動域[rad]
double	EnpArc_dof6c3_jangle_min[] = { -170 * DEG2RAD,	-160 * DEG2RAD,	-51 * DEG2RAD,	-200 * DEG2RAD,	-135 * DEG2RAD,	-360 * DEG2RAD };
double	EnpArc_dof6c3_jangle_max[] = { 170 * DEG2RAD,	  65 * DEG2RAD,	225 * DEG2RAD,	 200 * DEG2RAD,	 135 * DEG2RAD,	 360 * DEG2RAD };

PATHCOEF g_PathCoef0[6];


#define	IKNUM	0		//逆運動学の解の番号

#define	SINT	1.0		//加速時間

//************
// For Log
//************
#ifdef LOG_ANGLE
double LogJntAngle_Real[20100][6] = { 0.0 };	//実際の関節角度
double LogJntAngle_Des[20100][6] = { 0.0 };	//目標の関節角度
double Torque[20100][6] = { 0.0 };	//ロボットに入力した関節トルク
static long LogTick = 0;


#endif

#define SENSOR

#ifdef SENSOR
#define NAME_MEMCONT_SENSOR "Mem_SENSOR" 
#define NAME_SEMMEM_SENSOR "Sem_SENSOR" 
#endif

char	mess[128];
RTHANDLE	hProc, hMem, hSem;
RTHANDLE	hSemTraj, hSemSync;
void *		pMem;
SHAREDMEM*	pSharedMem;		// 共有メモリのアドレス shared memory address
BOOL	bFinish;
int		i;
DWORD	dwCount;
DWORD	dwOldContCount, dwContCount, dwOverrunCount;


VECTOR	*P01, *Theta01;

VECTOR	*n, *t, *b, *p;

double	jangle[20][10];
int	flg[20];

double	oldAng0[6];
double	tmp_cmang[6], tmp_cmvel[6], tmp_cmacc[6];
double	FirstJnt0[7];
double	TargetJnt0[7];
double	desTime;
int		FinishCount;
double	CurrentTime;

//ログ用
static char fname1[100];
static char fname2[100];
static FILE *fp;
static int g_i = 0;

double xp, yp, zp;

// 文字列の文字数をカウント count number of characters in the string
int charcount(char* ptrStr)
{
	char *ps;
	int	count;
	ps = ptrStr;
	count = 0;
	while (*ps != '\0') {
		ps++;
		count++;
	}
	return count;
}

// メッセージボックスへの書き出しサブルーチン write subroutine to message box
BOOL	SendMess(char *szText)
{
	return SendRtData(hMailTraj, szText, charcount(szText) + 1);
}


void EnpArc_dof6c3_6dof_inv_kine(VECTOR *n, VECTOR *t, VECTOR *b, VECTOR *p, double jangle0[], double jangle[20][10], int flg[]) {
	int	i, j;
	double	p0, p1, p2;
	VECTOR	w, wp, wpp, pp;
	double	s0, s1, s2, s3, s4, s12;
	double	c0, c1, c2, c3, c4, c12;
	double	a0, a1;
	double	d0, d3, d5;


	//全フラグをセット set all flags
	for (i = 0;i<8;i++)
		flg[i] = 1;

	//DH記法のパラメータからリンクの長さを正の値にする starting from the DH parameters make the length of links positive
	a0 = fabs(EnpArc_dof6c3_a[0]);
	a1 = fabs(EnpArc_dof6c3_a[1]);

	d0 = fabs(EnpArc_dof6c3_d[0]);
	d3 = fabs(EnpArc_dof6c3_d[3]);
	d5 = fabs(EnpArc_dof6c3_d[5]);

	/***** J1の角度を算出 calculate angle of J1*****/
	w.x = d5*b->x + p->x;
	w.y = d5*b->y + p->y;
	w.z = d5*b->z + p->z;

	for (i = 0;i<8;i++) {
		if (((i >> 2) & 0x1) == 0)	jangle[i][0] = (w.x == 0 && w.y == 0) ? jangle0[0] : atan2(w.y, w.x);		//←wがベースの真上に位置する時はJ1は前の角度を保持
		else						jangle[i][0] = (w.x == 0 && w.y == 0) ? jangle0[0] + PI : atan2(w.y, w.x) + PI;
	}

	//定義域を-π～πに変換 convert in the -π～π domain
	for (i = 0;i<8;i++)
		jangle[i][0] = (jangle[i][0]<-PI) ? 2 * PI + jangle[i][0] :
		(jangle[i][0]>PI) ? jangle[i][0] - 2 * PI : jangle[i][0];

	/***** J2及びJ3の角度を算出 calculate angles of J2 and J3*****/
	for (i = 0;i<8;i += 4) {
		s0 = sin(jangle[i][0]);
		c0 = cos(jangle[i][0]);

		wp.x = w.x*c0 + w.y*s0 - a0;
		wp.y = -w.x*s0 + w.y*c0;
		wp.z = w.z - d0;

		p0 = sqrt(wp.x*wp.x + wp.z*wp.z);

		if (p0>2 * a1) {	//先端が届かない特異点
			jangle[i][1] = 10000;
			jangle[i][2] = 10000;

			jangle[i + 1][1] = 10000;
			jangle[i + 1][2] = 10000;

			jangle[i + 2][1] = 10000;
			jangle[i + 2][2] = 10000;

			jangle[i + 3][1] = 10000;
			jangle[i + 3][2] = 10000;

			//フラグをリセット reset the flags
			flg[i] = 0;
			flg[i + 1] = 0;
			flg[i + 2] = 0;
			flg[i + 3] = 0;
		}
		else {
			p1 = atan2(wp.z, wp.x);
			p2 = acos(p0 / 2. / a1);

			jangle[i][1] = p1 + p2 - PI / 2.;
			jangle[i][2] = PI / 2. - 2 * p2;

			//定義域を-π～πに変換
			jangle[i][1] = (jangle[i][1]<-PI) ? 2 * PI + jangle[i][1] :
				(jangle[i][1]>PI) ? jangle[i][1] - 2 * PI : jangle[i][1];

			jangle[i + 1][1] = jangle[i][1];
			jangle[i + 1][2] = jangle[i][2];

			jangle[i + 2][1] = p1 - p2 - PI / 2.;
			jangle[i + 2][2] = PI / 2. + 2 * p2;

			//定義域を-π～πに変換
			jangle[i + 2][1] = (jangle[i + 2][1]<-PI) ? 2 * PI + jangle[i + 2][1] :
				(jangle[i + 2][1]>PI) ? jangle[i + 2][1] - 2 * PI : jangle[i + 2][1];

			jangle[i + 3][1] = jangle[i + 2][1];
			jangle[i + 3][2] = jangle[i + 2][2];
		}
	}

	/***** J4及びJ5の角度を算出 *****/
	for (i = 0;i<8;i += 2) {
		s0 = sin(jangle[i][0]);
		c0 = cos(jangle[i][0]);

		s1 = sin(jangle[i][1]);
		c1 = cos(jangle[i][1]);

		s2 = sin(jangle[i][2]);
		c2 = cos(jangle[i][2]);

		pp.x = c2*((p->x*c0 + p->y*s0 - a0)*c1 + (p->z - d0)*s1)
			+ s2*(-(p->x*c0 + p->y*s0 - a0)*s1 + (p->z - d0)*c1 - a1) - d3;
		pp.y = -p->x*s0 + p->y*c0;
		pp.z = -s2*((p->x*c0 + p->y*s0 - a0)*c1 + (p->z - d0)*s1)
			+ c2*(-(p->x*c0 + p->y*s0 - a0)*s1 + (p->z - d0)*c1 - a1);

		jangle[i][4] = acos(pp.x / sqrt(pp.x*pp.x + pp.y*pp.y + pp.z*pp.z));
		jangle[i][3] = (jangle[i][4] == 0) ? jangle0[3] : atan2(pp.y, pp.z);

		//定義域を-π～πに変換
		jangle[i][3] = (jangle[i][3]<-PI) ? 2 * PI + jangle[i][3] :
			(jangle[i][3]>PI) ? jangle[i][3] - 2 * PI : jangle[i][3];

		jangle[i + 1][4] = -jangle[i][4];
		jangle[i + 1][3] = (jangle[i + 1][4] == 0) ? jangle0[3] : jangle[i][3] - PI;

		//定義域を-π～πに変換
		jangle[i + 1][3] = (jangle[i + 1][3]<-PI) ? 2 * PI + jangle[i + 1][3] :
			(jangle[i + 1][3]>PI) ? jangle[i + 1][3] - 2 * PI : jangle[i + 1][3];
	}

	/***** J6の角度を算出 *****/
	for (i = 0;i<8;i += 2) {
		s0 = sin(jangle[i][0]);
		c0 = cos(jangle[i][0]);

		s12 = sin(jangle[i][1] + jangle[i][2]);
		c12 = cos(jangle[i][1] + jangle[i][2]);

		s3 = sin(jangle[i][3]);
		c3 = cos(jangle[i][3]);

		s4 = sin(jangle[i][4]);
		c4 = cos(jangle[i][4]);

		wpp.x = (c0*s12*c3 + s0*s3)*c4 + c0*c12*s4;
		wpp.y = (s0*s12*c3 - c0*s3)*c4 + s0*c12*s4;
		wpp.z = s12*s4 - c12*c3*c4;

		p0 = wpp.x*n->x + wpp.y*n->y + wpp.z*n->z;
		p0 = (p0>1) ? 1 : (p0<-1) ? -1 : p0;

		wp.x = wpp.y*n->z - wpp.z*n->y;
		wp.y = wpp.z*n->x - wpp.x*n->z;
		wp.z = wpp.x*n->y - wpp.y*n->x;

		p1 = wp.x*b->x + wp.y*b->y + wp.z*b->z;

		if (p1>0)	jangle[i][5] = acos(p0);
		else		jangle[i][5] = -acos(p0);

		jangle[i + 1][5] = jangle[i][5] - PI;

		//定義域を-π～πに変換
		jangle[i + 1][5] = (jangle[i + 1][5]<-PI) ? 2 * PI + jangle[i + 1][5] :
			(jangle[i + 1][5]>PI) ? jangle[i + 1][5] - 2 * PI : jangle[i + 1][5];
	}

	//可動範囲のチェック
	for (i = 0;i<8;i++) {
		for (j = 0;j<6;j++) {
			if (jangle[i][j]<EnpArc_dof6c3_jangle_min[j] || EnpArc_dof6c3_jangle_max[j]<jangle[i][j])
				flg[i] = 0;
		}
	}
}

void EnpArc_CalcRotMat(VECTOR *Theta, double m[3][3]) {
	double	sx, cx;
	double	sy, cy;
	double	sz, cz;

	sx = sin(Theta->x*DEG2RAD);
	cx = cos(Theta->x*DEG2RAD);

	sy = sin(Theta->y*DEG2RAD);
	cy = cos(Theta->y*DEG2RAD);

	sz = sin(Theta->z*DEG2RAD);
	cz = cos(Theta->z*DEG2RAD);

	m[0][0] = cy*cz;
	m[0][1] = -cy*sz;
	m[0][2] = sy;

	m[1][0] = sx*sy*cz + cx*sz;
	m[1][1] = -sx*sy*sz + cx*cz;
	m[1][2] = -sx*cy;

	m[2][0] = -cx*sy*cz + sx*sz;
	m[2][1] = cx*sy*sz + sx*cz;
	m[2][2] = cx*cy;
}

void EnpArc_MatMul(double m1[3][3], double m2[3][3]) { //matrix multiplication, save result in m2
	int	i, j, k;
	double m[3][3];

	for (i = 0;i<3;i++) {
		for (j = 0;j<3;j++) {
			m[i][j] = 0;
			for (k = 0;k<3;k++)
				m[i][j] += m1[i][k] * m2[k][j];
		}
	}

	for (i = 0;i<3;i++) {
		for (j = 0;j<3;j++) {
			m2[i][j] = m[i][j];
		}
	}
}
void EnpArc_MatVecMul(double m[3][3], double v[3]) { //m*v, save result in v
	int	i, k;
	double v0[3];

	for (i = 0;i<3;i++) {
		v0[i] = 0;
		for (k = 0;k<3;k++)
			v0[i] += m[i][k] * v[k];
	}

	for (i = 0;i<3;i++) {
		v[i] = v0[i];
	}
}
//w2k:work to kinematics coordinates rh:right hand
double EnpArc_trans_w2k_rh(VECTOR *P, VECTOR *Theta, double Delta,
	VECTOR *n, VECTOR *t, VECTOR *b, VECTOR *p) {
	double	m0[3][3], m1[3][3];
	double	v[3];

	//ベースの回転行列を設定 set pace rotation matrix
	m0[0][0] = 0;
	m0[1][0] = 1;
	m0[2][0] = 0;

	m0[0][1] = 1;
	m0[1][1] = 0;
	m0[2][1] = 0;

	m0[0][2] = 0;
	m0[1][2] = 0;
	m0[2][2] = -1;

	//運動学座標における回転行列を設定 set rotation matrix in kinematics coordinates
	EnpArc_CalcRotMat(Theta, m1);

	//運動学座標における手先位置を作業座標に変換 convert hand position from kinematics coordinates to work coordinates
	v[0] = P->x;
	v[1] = P->y;
	v[2] = P->z;

	EnpArc_MatVecMul(m0, v);

	p->x = v[0] / 1000.;
	p->y = v[1] / 1000.;
	p->z = v[2] / 1000.;

	//回転行列を変換 convert rotation matrix
	EnpArc_MatMul(m0, m1);

	//エンドポイントの回転行列を設定 set rotation matrix of the endpoint
	m0[0][0] = 0;
	m0[1][0] = -1;
	m0[2][0] = 0;

	m0[0][1] = -1;
	m0[1][1] = 0;
	m0[2][1] = 0;

	m0[0][2] = 0;
	m0[1][2] = 0;
	m0[2][2] = -1;

	EnpArc_MatMul(m1, m0);

	//運動学座標における姿勢を設定 set posture in kinematics coordinates
	n->x = m0[0][0];
	n->y = m0[1][0];
	n->z = m0[2][0];

	t->x = m0[0][1];
	t->y = m0[1][1];
	t->z = m0[2][1];

	b->x = m0[0][2];
	b->y = m0[1][2];
	b->z = m0[2][2];

	//作業座標におけるδを運動学座標に変換 convert δ from work to kinematics coordinates
	return	-Delta*DEG2RAD;
}
//w2k:work to kinematics coordinates lh:left hand
double EnpArc_trans_w2k_lh(VECTOR *P, VECTOR *Theta, double Delta,
	VECTOR *n, VECTOR *t, VECTOR *b, VECTOR *p) {
	double	m0[3][3], m1[3][3];
	double	v[3];

	//ベースの回転行列を設定
	m0[0][0] = 0;
	m0[1][0] = -1;
	m0[2][0] = 0;

	m0[0][1] = 1;
	m0[1][1] = 0;
	m0[2][1] = 0;

	m0[0][2] = 0;
	m0[1][2] = 0;
	m0[2][2] = -1;

	//運動学座標における回転行列を設定
	EnpArc_CalcRotMat(Theta, m1);

	//運動学座標における手先位置を作業座標に変換
	v[0] = P->x;
	v[1] = P->y;
	v[2] = P->z;

	EnpArc_MatVecMul(m0, v);

	p->x = v[0] / 1000.;
	p->y = v[1] / 1000.;
	p->z = v[2] / 1000.;

	//回転行列を変換
	EnpArc_MatMul(m0, m1);

	//エンドポイントの回転行列を設定
	m0[0][0] = 0;
	m0[1][0] = -1;
	m0[2][0] = 0;

	m0[0][1] = 1;
	m0[1][1] = 0;
	m0[2][1] = 0;

	m0[0][2] = 0;
	m0[1][2] = 0;
	m0[2][2] = -1;

	EnpArc_MatMul(m1, m0);

	//運動学座標における姿勢を設定
	n->x = m0[0][0];
	n->y = m0[1][0];
	n->z = m0[2][0];

	t->x = m0[0][1];
	t->y = m0[1][1];
	t->z = m0[2][1];

	b->x = m0[0][2];
	b->y = m0[1][2];
	b->z = m0[2][2];

	//作業座標におけるδを運動学座標に変換
	return	Delta*DEG2RAD;
}


// 5次多項式の係数計算
void endpointmode_PathInit_j(double FirstAng[], double TargetAng[], double DesTime)
{
	int  i;
	double t3, t4, t5;
	double diff[6];

	t3 = DesTime * DesTime * DesTime;
	t4 = t3 * DesTime;
	t5 = t4 * DesTime;


	for (i = 0; i < 6; i++) {
		diff[i] = TargetAng[i] - FirstAng[i];
		g_PathCoef0[i].pos0 = FirstAng[i];
		g_PathCoef0[i].pos3 = 10. * diff[i] / t3;
		g_PathCoef0[i].pos4 = -15. * diff[i] / t4;
		g_PathCoef0[i].pos5 = 6. * diff[i] / t5;

		g_PathCoef0[i].vel2 = 3. * g_PathCoef0[i].pos3;
		g_PathCoef0[i].vel3 = 4. * g_PathCoef0[i].pos4;
		g_PathCoef0[i].vel4 = 5. * g_PathCoef0[i].pos5;
		g_PathCoef0[i].acc1 = 6. * g_PathCoef0[i].pos3;
		g_PathCoef0[i].acc2 = 12. * g_PathCoef0[i].pos4;
		g_PathCoef0[i].acc3 = 20. * g_PathCoef0[i].pos5;
	}

}

// 目標角度，速度，加速度の逐次計算 
void endpointmode_PathGenerate_j(double CmAng[6], double CmVel[6], double CmAcc[6], double DesTime, double CountTime)
{

	int i;
	double t, t2, t3, t4, t5;

	t = (CountTime > DesTime) ? DesTime : CountTime;

	t2 = t * t;
	t3 = t2 * t;
	t4 = t3 * t;
	t5 = t4 * t;


	for (i = 0; i < 6; i++) {
		CmAng[i] = g_PathCoef0[i].pos0
			+ (t3 * g_PathCoef0[i].pos3)
			+ (t4 * g_PathCoef0[i].pos4)
			+ (t5 * g_PathCoef0[i].pos5);
		CmVel[i] = (t2 * g_PathCoef0[i].vel2)
			+ (t3 * g_PathCoef0[i].vel3)
			+ (t4 * g_PathCoef0[i].vel4);
		CmAcc[i] = (t * g_PathCoef0[i].acc1)
			+ (t2 * g_PathCoef0[i].acc2)
			+ (t3 * g_PathCoef0[i].acc3);
	}

}
// 関節角速度が制限速度を超えていないかをチェック
int endpointmode_CheckAngSpeed(double AngVel[])
{
	int i, err;

	err = 0;

	for (i = 0; i < 6; i++) {
		if (fabs(AngVel[i] * RAD2DEG) >(EnpArc_JntMaxSpd[i] * 0.5)) {
			//			printf("Joint[%d] is Speed Over !!\n", i+1);
			err++;
		}
	}

	if (err != 0) {
		return -1;
	}

	return 1;
}
/*+++++関数定義終了+++++*/


/*****************************************************************************
*
* FUNCTION:		Trajectory
*
* PARAMETERS:	なし none
*
* RETURNS:		なし none
*
* DESCRIPTION:  軌道生成処理スレッド orbits generation
*****************************************************************************/



		

void			Trajectory(void)
{
	//	WORD	messcount;

	//メモリを確保
	P01 = (VECTOR *)malloc(sizeof(VECTOR));
	Theta01 = (VECTOR *)malloc(sizeof(VECTOR));


	n = (VECTOR *)malloc(sizeof(VECTOR));
	t = (VECTOR *)malloc(sizeof(VECTOR));
	b = (VECTOR *)malloc(sizeof(VECTOR));
	p = (VECTOR *)malloc(sizeof(VECTOR));

#ifdef SENSOR
	void* pMemCont_SENSOR;
	RTHANDLE hMemCont_SENSOR;
	RTHANDLE hSemMem_SENSOR;
	SHAREDMEM_SENSOR* pSharedMem_SENSOR;
#endif

	SendMess("Trajectory Creation Process started");
	// 制御インターバル処理の検索
	hProc = LookupRtHandle(hRootProcess, NAME_PROCCONT, WAIT_FOREVER);
	if (hProc == BAD_RTHANDLE)
		Fail("ProcTraj: Cannot find shared memory process MemCont");
	// 共有メモリの検索とマッピング
	hMem = LookupRtHandle(hProc, NAME_MEMCONT, WAIT_FOREVER);
	if (hMem == BAD_RTHANDLE)
		Fail("ProcTraj: Cannot find shared memory MemCont");
	pMem = MapRtSharedMemory(hMem);
	if (pMem == NULL)
		Fail("ProcTraj: Cannot map shared memory MemCont");
	pSharedMem = pMem;
	// メモリ排他制御用セマフォの検索
	hSem = LookupRtHandle(hProc, NAME_SEMMEM, WAIT_FOREVER);
	if (hSem == BAD_RTHANDLE)
		Fail("ProcTraj: Cannot find semaphore SemMem");
	// 制御インターバルセマフォの検索
	hSemTraj = LookupRtHandle(hProc, NAME_SEMTRAJ, WAIT_FOREVER);
	if (hSemTraj == BAD_RTHANDLE)
		Fail("TrajProc: Cannot find semaphore SemTraj");
	// 制御インターバル同期用セマフォの検索
	hSemSync = LookupRtHandle(hProc, NAME_SEMSYNC, WAIT_FOREVER);
	if (hSemSync == BAD_RTHANDLE)
		Fail("TrajProc: Cannot find semaphore SemSync");

	// 起動生成用セマフォの取得
	if (WaitForRtSemaphore(hSemTraj, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemTraj");
	}

	// 制御インターバル同期用セマフォの取得 ・・・ 1発めは2回取得する
	if (WaitForRtSemaphore(hSemSync, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemSync");
	}
	if (WaitForRtSemaphore(hSemSync, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemSync");
	}
	//	↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	//	ここから軌道計画処理を記述します。trajectory planning process from here

	//引数の処理 processing arguments
	//引数の処理

	bFinish = FALSE;
	dwCount = 0;
	dwOverrunCount = 0;

#ifdef SENSOR

	// 共有メモリの検索とマッピング
	hMemCont_SENSOR = LookupRtHandle(hProc, NAME_MEMCONT_SENSOR, WAIT_FOREVER);
	if (hMemCont_SENSOR == BAD_RTHANDLE)
		Fail("ProcTraj: Cannot find shared memory　Mem_SENSOR");
	pMemCont_SENSOR = MapRtSharedMemory(hMemCont_SENSOR);
	if (pMemCont_SENSOR == NULL)
		Fail("ProcTraj: Cannot map shared memory Mem_SENSOR");
	pSharedMem_SENSOR = (SHAREDMEM_SENSOR*)pMemCont_SENSOR;

	// //セマフォの検索
	// hSemMem_SENSOR = LookupRtHandle(hProc, NAME_SEMMEM_SENSOR, WAIT_FOREVER);
	// if (hSemMem_SENSOR == BAD_RTHANDLE)
	// 	Fail("ProcTraj: Cannot find semaphore Sem_SENSOR");


	// // メモリ用セマフォの取得
	// if (WaitForRtSemaphore(hSemMem_SENSOR, 1, WAIT_FOREVER) == WAIT_FAILED) {
	// 	Fail("ProcTraj: Cannot receive semaphore Sem_SENSOR");

	// 	//release semaphore for memory of image processing
	// 	ReleaseRtSemaphore(hSemMem_SENSOR, 1);

	// 	if (GetLastRtError() != E_OK)
	// 	{
	// 		ReleaseRtSemaphore(hSemMem_SENSOR, 0);
	// 	}
	// }

	for (i = 0; i < 6; i++) {
		TargetJnt0[i] = 0.;
	}
	for (i = 0; i < 6; i++) {
		TargetJnt0[i] = pSharedMem_SENSOR->SensorData[i];
	}
	sprintf(mess, "Received Angle\n");
	SendMess(mess);

	//セマフォのリリース
	ReleaseRtSemaphore(hSemMem_SENSOR, 1);
#endif
	
	xp = pSharedMem_SENSOR->SensorData[0];
	yp = pSharedMem_SENSOR->SensorData[1];
	zp = pSharedMem_SENSOR->SensorData[2];

	if (iArgc<8) {
		bFinish = TRUE;		
		pSharedMem_SENSOR->SensorData[3] == 0.0;
	}
	else {
		if (pSharedMem_SENSOR->SensorData[3] == 0.0f){
			sprintf(mess, "command now is %6.1f\n", pSharedMem_SENSOR->SensorData[3]);
			SendMess(mess);
		}
		else if (pSharedMem_SENSOR->SensorData[3] == 1.0f){
			P01->x = xp - atof(szArgv[1]); //x' same as x camera
			P01->y = atof(szArgv[3]) - zp; //z' z cam + 470
			P01->z = atof(szArgv[2]) - yp -175; //y' y cam -160
			Theta01->x = atof(szArgv[4]);
			Theta01->y = atof(szArgv[5]); //if else +30 -30 0
			Theta01->z = atof(szArgv[6]);
			desTime = atof(szArgv[7]);
		}

	}

	// 変数の初期化
	for (i = 0; i < 6; i++) {
		FirstJnt0[i] = 0.;
		tmp_cmang[i] = 0.;
		tmp_cmvel[i] = 0.;
		tmp_cmacc[i] = 0.;
	}

	// 目標時間が1s以下に指定された場合は，安全のため1sとする(マイナスの値などを除外)．
	if (desTime < 1.0) {
		desTime = 1.0;
	}

	// メモリ用セマフォの取得
	if (WaitForRtSemaphore(hSem, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemMem");
	}
	// サンプリング制御処理オーバーラン確認用カウント値取得
	dwOldContCount = pSharedMem->S_JMaster.FreeRunCount;
	// 軌道生成カウンタ初期値書き込み
	pSharedMem->C_JMaster.TrajCount = dwCount;
	// メモリ用セマフォの開放
	if (!ReleaseRtSemaphore(hSem, 1)) {
		Fail("ProcTraj: Cannot release semaphore SemMem");
	}

	//現在の関節角度値を取得
	for (i = 0; i<6; i++) {
		FirstJnt0[i] = pSharedMem->C_Joint[0][i].Position;
		oldAng0[i] = pSharedMem->S_Joint[0][i].Position;
	}

	//目標関節角度の計算 calculation of target joint angle
	//Arm0
	//右手系作業座標における位置姿勢を運動学座標に変換 convert position and orinetationin in right handed kinematics coordinates
	EnpArc_trans_w2k_rh(P01, Theta01, 0.0, n, t, b, p);


	//逆運動学を用いて関節角度を計算 calculate joint angle using inverse kinematics
	EnpArc_dof6c3_6dof_inv_kine(n, t, b, p, FirstJnt0, jangle, flg);


	//目標関節角度を設定 set the target joint angle
	for (i = 0;i<6;i++) {
		TargetJnt0[i] = jangle[IKNUM][i];
	}

	if (flg[IKNUM] != 1) {
		for (i = 0;i<6;i++)
			TargetJnt0[i] = FirstJnt0[i];

		sprintf(mess, "Arm0:特異姿勢です"); //singular posture!!
		SendMess(mess);

		bFinish = TRUE;
	}


	// 軌道の係数計算
	endpointmode_PathInit_j(FirstJnt0, TargetJnt0, desTime);


	//表示
	//初期位置・姿勢
	sprintf(mess, "ARM0\n");
	SendMess(mess);
	sprintf(mess, "  FirstAngle\t: %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f\n", FirstJnt0[0] * RAD2DEG, FirstJnt0[1] * RAD2DEG, FirstJnt0[2] * RAD2DEG, FirstJnt0[3] * RAD2DEG, FirstJnt0[4] * RAD2DEG, FirstJnt0[5] * RAD2DEG);
	SendMess(mess);
	sprintf(mess, "  TargetAngle\t: %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f\n", TargetJnt0[0] * RAD2DEG, TargetJnt0[1] * RAD2DEG, TargetJnt0[2] * RAD2DEG, TargetJnt0[3] * RAD2DEG, TargetJnt0[4] * RAD2DEG, TargetJnt0[5] * RAD2DEG);
	SendMess(mess);

	//動作時間
	sprintf(mess, "   DesTime = %8.2lf[sec]", desTime);
	SendMess(mess);

	CurrentTime = (double)0;
	FinishCount = 1;


	//	ここまで軌道計画処理 end of trajectory planning process
	//	↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

	while (!bFinish)
	{
		dwCount++;
		// 制御インターバル同期用セマフォの取得
		if (WaitForRtSemaphore(hSemSync, 1, WAIT_FOREVER) == WAIT_FAILED) {
			Fail("ProcTraj: Cannot receive semaphore SemSync");
		}

		//	↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
		//	ここから逐次の軌道生成処理を記述します。

		// メモリ用セマフォの取得
		if (WaitForRtSemaphore(hSem, 1, WAIT_FOREVER) == WAIT_FAILED) {
			Fail("ProcTraj: Cannot receive semaphore SemMem");
		}
		// ----------------- これ以上はいじらない

		// 逐次軌道生成処理 sequential trajectroy generation
		CurrentTime += SAMPLING_INTERVAL;

		// 1msec後の目標角度，目標速度，目標加速度を計算する
		endpointmode_PathGenerate_j(tmp_cmang, tmp_cmvel, tmp_cmacc, desTime, CurrentTime);

		// 目標の関節角速度が最大角速度超えていないかをチェック！
		//if ( endpointmode_CheckAngSpeed( tmp_cmvel ) != 1 ) {
		//	for (i = 0; i < 6; i++) {
		//		tmp_cmang[i] = oldAng0[i];
		//	}
		//	endpointmode_PathGenerate_j(tmp_cmang, tmp_cmvel, tmp_cmacc, desTime, CurrentTime );
		//}

		// 目標の関節角速度が最大角速度超えていないかをチェック！
		//2025年渡辺が変更
		//スピードが超過した場合強制終了するように変更
		//変えたばかりなのであまりあてにしすぎずに各自で超過しないよう気を付けてください
		if (endpointmode_CheckAngSpeed(tmp_cmvel) != 1) {
			for (i = 0; i < 6; i++) {

				tmp_cmang[i] = oldAng0[i];
				tmp_cmvel[i] = 0.;
				tmp_cmacc[i] = 0.;
			}
			//JointMode_PathGenerate_j(tmp_cmang, tmp_cmvel, tmp_cmacc, desTime, CurrentTime, TargetJnt0, FirstJnt0);
			bFinish = TRUE;
			sprintf(mess, "  error:speedover\n");
			SendMess(mess);

		}



		for (i = 0; i < 6; i++) {
			pSharedMem->C_Joint[0][i].Position = tmp_cmang[i];
			pSharedMem->C_Joint[0][i].Velocity = tmp_cmvel[i];
			pSharedMem->C_Joint[0][i].Accel = tmp_cmacc[i];
			oldAng0[i] = tmp_cmang[i];
		}


#ifdef LOG_ANGLE	
		//ログ取得
		if (LogTick<20000) {
			for (i = 0; i<6; i++) {
				LogJntAngle_Des[LogTick][i] = pSharedMem->C_Joint[0][i].Position;
			}
			//現在の各関節角度を取得
			for (i = 0; i<6; i++) {
				LogJntAngle_Real[LogTick][i] = pSharedMem->S_Joint[0][i].Position;
			}
			//入力関節トルク
			for (i = 0; i<6; i++) {
				Torque[LogTick][i] = pSharedMem->C_Joint[0][i].Torque;
			}
			LogTick++;
		}
#endif

		// 目標時間を過ぎたら終了！
		if (CurrentTime >= desTime+20) {
			FinishCount = 0;
		}

		if (FinishCount == 0) {
			bFinish = TRUE;
		}
		// ----------------- これ以下はいじらない
		pSharedMem->C_JMaster.TrajCount = dwCount;

		dwContCount = pSharedMem->S_JMaster.FreeRunCount;
		if (dwContCount == dwOldContCount)
			dwOverrunCount++;
		dwOldContCount = dwContCount;
		// メモリ用セマフォの開放
		if (!ReleaseRtSemaphore(hSem, 1)) {
			Fail("ProcTraj: Cannot release semaphore SemMem");
		}

		//	ここまで逐次の軌道生成処理 end of sequential trajectroy generation
		//	↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

		// 1000回毎にメールボックスへ書き込み
		count1000++;
		if (count1000 % 1000 == 0) {
			++sec;
			//			messcount = sprintf(mess, "ProcTraj: %ld[sec] Vel=%lf", sec, Vmain);
			//			if ( !SendRtData(hMailTraj, mess, ++messcount) )
			//				Fail("ProcTraj: Cannot send to data mailbox");
			count1000 = 0;
		}
	}
	//#if _DEBUG
#ifndef NDEBUG
	printf("TrajLoop = %ld\n", dwCount);
#endif
	// 制御インターバル同期用セマフォの取得
	if (WaitForRtSemaphore(hSemSync, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemSync");
	}
	/*
	↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	ここから軌道生成処理の後処理を記述します。postprocessing
	*/

	//---------------
	//+++ログの出力+++
	//---------------

#ifdef LOG_ANGLE


	sprintf(mess,"log start");
	SendMess(mess);
	sprintf(fname1, "C:\\Work_C5\\Work_pitchada\\Jnt.txt");
	for (g_i = 0; g_i < LogTick; g_i++) {
		fp = fopen(fname1, "a");	//オープン

		fprintf(fp, "%.16f ", LogJntAngle_Des[g_i][0] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Des[g_i][1] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Des[g_i][2] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Des[g_i][3] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Des[g_i][4] * RAD2DEG);
		fprintf(fp, "%.16f \t ", LogJntAngle_Des[g_i][5] * RAD2DEG);

		fprintf(fp, "%.16f ", LogJntAngle_Real[g_i][0] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Real[g_i][1] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Real[g_i][2] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Real[g_i][3] * RAD2DEG);
		fprintf(fp, "%.16f ", LogJntAngle_Real[g_i][4] * RAD2DEG);
		fprintf(fp, "%.16f \t ", LogJntAngle_Real[g_i][5] * RAD2DEG);


		// fprintf(fp, "%.16f ", Torque[g_i][0]);
		// fprintf(fp, "%.16f ", Torque[g_i][1]);
		// fprintf(fp, "%.16f ", Torque[g_i][2]);
		// fprintf(fp, "%.16f ", Torque[g_i][3]);
		// fprintf(fp, "%.16f ", Torque[g_i][4]);
		// fprintf(fp, "%.16f\n ", Torque[g_i][5]);

		fclose(fp);


	}
#endif

	//==ログ出力の終了==

	//メモリを開放
	free(P01);
	free(Theta01);


	free(n);
	free(t);
	free(b);
	free(p);

	// メモリ用セマフォの取得
	if (WaitForRtSemaphore(hSem, 1, WAIT_FOREVER) == WAIT_FAILED) {
		Fail("ProcTraj: Cannot receive semaphore SemMem");
	}
	// 軌道生成カウンタ最終書き込み
	pSharedMem->C_JMaster.TrajCount = 0xFFFFFFFF;
	// メモリ用セマフォの開放
	if (!ReleaseRtSemaphore(hSem, 1)) {
		Fail("ProcTraj: Cannot release semaphore SemMem");
	}
	// 起動生成後処理
	sprintf(mess, "   Total Overrun count is %ld", (int)dwOverrunCount);
	SendMess(mess);
	/*
	#ifdef _DEBUG
	printf("   Total Overrun count is %ld\n", (int)dwOverrunCount);
	#endif
	*/
	//	ここまで軌道生成処理の後処理
	//	↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

	// 起動生成用セマフォの開放
	if (!ReleaseRtSemaphore(hSemTraj, 1)) {
		Fail("TrajProc: Cannot release semaphore SemTraj");
	}
	SendMess("Trajectory Creation Process finished");

	// WindowsからRTスレッドを終了してもらうための処理
	if (!SendRtData(hMailTraj, "end", 4))
		Fail("ProcTraj: Cannot send to data mailbox");

	/*
	#ifdef _DEBUG
	printf("Trajectory Creation Process finished\n");
	#endif
	*/
	SuspendRtThread(NULL_RTHANDLE);
}

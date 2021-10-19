//#include "filter.h"

//#define numStages  2                /* 2阶IIR滤波的个数 */
//#define TEST_LENGTH_SAMPLES  400    /* 采样点数 */
//#define BLOCK_SIZE           1    /* 调用一次arm_biquad_cascade_df1_f32处理的采样点个数 */


//uint32_t blockSize = BLOCK_SIZE;
//uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;      /* 需要调用arm_biquad_cascade_df1_f32的次数 */


//static fp32 testInput_f32_50Hz_200Hz[TEST_LENGTH_SAMPLES]; /* 采样点 */
//static fp32 testOutput[TEST_LENGTH_SAMPLES];               /* 滤波后的输出 */
//static fp32 IIRStateF32[4*numStages];                      /* 状态缓存 */
//      
///* 巴特沃斯低通滤波器系数 80Hz*/                                                                                                                                         
//const fp32 IIRCoeffs32LP[5*numStages] = {                                                                                 
//    1.0f,  2.0f,  1.0f,  1.479798894397216679763573665695730596781f,  
//-0.688676953053861784503908438637154176831f,

//    1.0f,  2.0f,  1.0f,  1.212812092620218384908525877108331769705f,  
//-0.384004162286553540894828984164632856846f                                                                                              
//};                                               

///*
//*********************************************************************************************************
//*    函 数 名: arm_iir_f32_lp
//*    功能说明: 调用函数arm_iir_f32_lp实现低通滤波器
//*    形    参：无
//*    返 回 值: 无
//*********************************************************************************************************
//*/
//static void arm_iir_f32_lp(void)
//{
//    uint32_t i;
//    arm_biquad_casd_df1_inst_f32 S;
//    fp32 ScaleValue;
//    fp32  *inputF32, *outputF32;
//    
//    /* 初始化输入输出缓存指针 */
//    inputF32 = &testInput_f32_50Hz_200Hz[0];
//    outputF32 = &testOutput[0];
//    
//    
//    /* 初始化 */
//    arm_biquad_cascade_df1_init_f32(&S, numStages, (fp32 *)&IIRCoeffs32LP[0], (fp32 *)&IIRStateF32[0]);
//    
//    /* 实现IIR滤波，这里每次处理1个点 */
//    for(i=0; i < numBlocks; i++)
//    {
//        arm_biquad_cascade_df1_f32(&S, inputF32 + (i * blockSize),  outputF32 + (i * blockSize), blockSize);
//    }
//            
//    /*放缩系数 */
//    ScaleValue = 0.052219514664161220673932461977528873831f * 0.042798017416583809813257488485760404728f ; 
//    
//    /* 打印滤波后结果 */
//    for(i=0; i<TEST_LENGTH_SAMPLES; i++)
//    {
//        printf("line3=%f, line4=%f\r\n", testInput_f32_50Hz_200Hz[i], testOutput[i]*ScaleValue);
//    }
//}

///**
// *  *S       ：指向浮点Biquad级联结构的实例.
// *  *pSrc    ：指向输入数据块。
// *  *pDst    ：指向输出数据块。
// *  blockSize：每次调用要处理的样本数。
// *  返回值    ：无.
// */
//void arm_biquad_cascade_df1_f32(
//  const arm_biquad_casd_df1_inst_f32 * S,
//  float * pSrc,
//  float * pDst,
//  unsigned int blockSize)
//{
//  float *pIn = pSrc;                         /*源指针     */
//  float *pOut = pDst;                        /*目的指针    */
//  float *pState = S->pState;                 /*状态指针    */
//  float *pCoeffs = S->pCoeffs;               /*参数指针    */
//  float acc;                                 /*累加器      */
//  float b0, b1, b2, a1, a2;                  /*滤波器参数   */
//  float Xn1, Xn2, Yn1, Yn2;                  /*滤波器状态变量*/
//  float Xn;                                  /*临时输入     */
//  unsigned int sample, stage = S->numStages; /*循环计数     */

//  do
//  {
//    /* Reading the coefficients */
//    b0 = *pCoeffs++;
//    b1 = *pCoeffs++;
//    b2 = *pCoeffs++;
//    a1 = *pCoeffs++;
//    a2 = *pCoeffs++;

//    Xn1 = pState[0];
//    Xn2 = pState[1];
//    Yn1 = pState[2];
//    Yn2 = pState[3];

//    sample = blockSize >> 2u;

//    while(sample > 0u)
//    {
//      /* 读第一个输入 */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn2 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

//      /* Store the result in the accumulator in the destination buffer. */
//      *pOut++ = Yn2;

//      /* 每次计算输出后，状态都应更新. */
//      /* 状态应更新为:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */

//      /* Read the second input */
//      Xn2 = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn1 = (b0 * Xn2) + (b1 * Xn) + (b2 * Xn1) + (a1 * Yn2) + (a2 * Yn1);

//      /* 将结果存储在目标缓冲区的累加器中. */
//      *pOut++ = Yn1;

//      /* 每次计算输出后，状态都应更新. */
//      /* 状态应更新为:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */

//      /*读第三个输入 */
//      Xn1 = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn2 = (b0 * Xn1) + (b1 * Xn2) + (b2 * Xn) + (a1 * Yn1) + (a2 * Yn2);

//      /* 将结果存储在目标缓冲区的累加器中. */
//      *pOut++ = Yn2;

//      /* 每次计算输出后，状态都应更新. */
//      /* 状态应更新为: */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      /* 读第四个输入 */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn1 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn2) + (a2 * Yn1);

//      /* 将结果存储在目标缓冲区的累加器中. */
//      *pOut++ = Yn1;

//      /* 每次计算输出后，状态都应更新. */
//      /* 状态应更新为:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      Xn2 = Xn1;
//      Xn1 = Xn;

//      /* 递减循环计数器 */
//      sample--;
//    }

//    /* 如果blockSize不是4的倍数，
//    *请在此处计算任何剩余的输出样本。
//    *不使用循环展开. */
//    sample = blockSize & 0x3u;

//    while(sample > 0u)
//    {
//      /* 读取输入 */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

//      /* 将结果存储在目标缓冲区的累加器中. */
//      *pOut++ = acc;

//      /* 每次计算输出后，状态都应更新。 */
//      /* 状态应更新为:    */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      Xn2 = Xn1;
//      Xn1 = Xn;
//      Yn2 = Yn1;
//      Yn1 = acc;

//      /* d递减循环计数器 */
//      sample--;
//    }

//    /*  将更新后的状态变量存储回pState数组中 */
//    *pState++ = Xn1;
//    *pState++ = Xn2;
//    *pState++ = Yn1;
//    *pState++ = Yn2;

//    /*第一阶段从输入缓冲区到输出缓冲区.     */
//    /*随后的numStages在输出缓冲区中就地发生*/
//    pIn = pDst;

//    /* 重置输出指针 */
//    pOut = pDst;

//    /* 递减循环计数器 */
//    stage--;

//  } while(stage > 0u);
//}

///*
//*作用      :初始化滤波器
//*S        :指向浮点SOS级联结构的实例。
//*numStages:滤波器中二阶SOS的数量
//*pCoeffs  :滤波器参数指针,参数按下列顺序存储
//*          {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
//*pState   :历史状态缓冲区指针
//*/
//void arm_biquad_cascade_df1_init_f32(
//        arm_biquad_casd_df1_inst_f32 * S,
//        uint8_t numStages,
//  const fp32 * pCoeffs,
//        fp32 * pState)
//{
//  /* Assign filter stages */
//  S->numStages = numStages;

//  /* Assign coefficient pointer */
//  S->pCoeffs = pCoeffs;

//  /* Clear state buffer and size is always 4 * numStages */
//  memset(pState, 0, (4U * (uint32_t) numStages) * sizeof(fp32));

//  /* Assign state pointer */
//  S->pState = pState;
//}
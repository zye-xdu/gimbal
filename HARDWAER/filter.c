//#include "filter.h"

//#define numStages  2                /* 2��IIR�˲��ĸ��� */
//#define TEST_LENGTH_SAMPLES  400    /* �������� */
//#define BLOCK_SIZE           1    /* ����һ��arm_biquad_cascade_df1_f32����Ĳ�������� */


//uint32_t blockSize = BLOCK_SIZE;
//uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;      /* ��Ҫ����arm_biquad_cascade_df1_f32�Ĵ��� */


//static fp32 testInput_f32_50Hz_200Hz[TEST_LENGTH_SAMPLES]; /* ������ */
//static fp32 testOutput[TEST_LENGTH_SAMPLES];               /* �˲������� */
//static fp32 IIRStateF32[4*numStages];                      /* ״̬���� */
//      
///* ������˹��ͨ�˲���ϵ�� 80Hz*/                                                                                                                                         
//const fp32 IIRCoeffs32LP[5*numStages] = {                                                                                 
//    1.0f,  2.0f,  1.0f,  1.479798894397216679763573665695730596781f,  
//-0.688676953053861784503908438637154176831f,

//    1.0f,  2.0f,  1.0f,  1.212812092620218384908525877108331769705f,  
//-0.384004162286553540894828984164632856846f                                                                                              
//};                                               

///*
//*********************************************************************************************************
//*    �� �� ��: arm_iir_f32_lp
//*    ����˵��: ���ú���arm_iir_f32_lpʵ�ֵ�ͨ�˲���
//*    ��    �Σ���
//*    �� �� ֵ: ��
//*********************************************************************************************************
//*/
//static void arm_iir_f32_lp(void)
//{
//    uint32_t i;
//    arm_biquad_casd_df1_inst_f32 S;
//    fp32 ScaleValue;
//    fp32  *inputF32, *outputF32;
//    
//    /* ��ʼ�������������ָ�� */
//    inputF32 = &testInput_f32_50Hz_200Hz[0];
//    outputF32 = &testOutput[0];
//    
//    
//    /* ��ʼ�� */
//    arm_biquad_cascade_df1_init_f32(&S, numStages, (fp32 *)&IIRCoeffs32LP[0], (fp32 *)&IIRStateF32[0]);
//    
//    /* ʵ��IIR�˲�������ÿ�δ���1���� */
//    for(i=0; i < numBlocks; i++)
//    {
//        arm_biquad_cascade_df1_f32(&S, inputF32 + (i * blockSize),  outputF32 + (i * blockSize), blockSize);
//    }
//            
//    /*����ϵ�� */
//    ScaleValue = 0.052219514664161220673932461977528873831f * 0.042798017416583809813257488485760404728f ; 
//    
//    /* ��ӡ�˲����� */
//    for(i=0; i<TEST_LENGTH_SAMPLES; i++)
//    {
//        printf("line3=%f, line4=%f\r\n", testInput_f32_50Hz_200Hz[i], testOutput[i]*ScaleValue);
//    }
//}

///**
// *  *S       ��ָ�򸡵�Biquad�����ṹ��ʵ��.
// *  *pSrc    ��ָ���������ݿ顣
// *  *pDst    ��ָ��������ݿ顣
// *  blockSize��ÿ�ε���Ҫ�������������
// *  ����ֵ    ����.
// */
//void arm_biquad_cascade_df1_f32(
//  const arm_biquad_casd_df1_inst_f32 * S,
//  float * pSrc,
//  float * pDst,
//  unsigned int blockSize)
//{
//  float *pIn = pSrc;                         /*Դָ��     */
//  float *pOut = pDst;                        /*Ŀ��ָ��    */
//  float *pState = S->pState;                 /*״ָ̬��    */
//  float *pCoeffs = S->pCoeffs;               /*����ָ��    */
//  float acc;                                 /*�ۼ���      */
//  float b0, b1, b2, a1, a2;                  /*�˲�������   */
//  float Xn1, Xn2, Yn1, Yn2;                  /*�˲���״̬����*/
//  float Xn;                                  /*��ʱ����     */
//  unsigned int sample, stage = S->numStages; /*ѭ������     */

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
//      /* ����һ������ */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn2 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

//      /* Store the result in the accumulator in the destination buffer. */
//      *pOut++ = Yn2;

//      /* ÿ�μ��������״̬��Ӧ����. */
//      /* ״̬Ӧ����Ϊ:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */

//      /* Read the second input */
//      Xn2 = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn1 = (b0 * Xn2) + (b1 * Xn) + (b2 * Xn1) + (a1 * Yn2) + (a2 * Yn1);

//      /* ������洢��Ŀ�껺�������ۼ�����. */
//      *pOut++ = Yn1;

//      /* ÿ�μ��������״̬��Ӧ����. */
//      /* ״̬Ӧ����Ϊ:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */

//      /*������������ */
//      Xn1 = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn2 = (b0 * Xn1) + (b1 * Xn2) + (b2 * Xn) + (a1 * Yn1) + (a2 * Yn2);

//      /* ������洢��Ŀ�껺�������ۼ�����. */
//      *pOut++ = Yn2;

//      /* ÿ�μ��������״̬��Ӧ����. */
//      /* ״̬Ӧ����Ϊ: */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      /* �����ĸ����� */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      Yn1 = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn2) + (a2 * Yn1);

//      /* ������洢��Ŀ�껺�������ۼ�����. */
//      *pOut++ = Yn1;

//      /* ÿ�μ��������״̬��Ӧ����. */
//      /* ״̬Ӧ����Ϊ:  */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      Xn2 = Xn1;
//      Xn1 = Xn;

//      /* �ݼ�ѭ�������� */
//      sample--;
//    }

//    /* ���blockSize����4�ı�����
//    *���ڴ˴������κ�ʣ������������
//    *��ʹ��ѭ��չ��. */
//    sample = blockSize & 0x3u;

//    while(sample > 0u)
//    {
//      /* ��ȡ���� */
//      Xn = *pIn++;

//      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
//      acc = (b0 * Xn) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);

//      /* ������洢��Ŀ�껺�������ۼ�����. */
//      *pOut++ = acc;

//      /* ÿ�μ��������״̬��Ӧ���¡� */
//      /* ״̬Ӧ����Ϊ:    */
//      /* Xn2 = Xn1    */
//      /* Xn1 = Xn     */
//      /* Yn2 = Yn1    */
//      /* Yn1 = acc   */
//      Xn2 = Xn1;
//      Xn1 = Xn;
//      Yn2 = Yn1;
//      Yn1 = acc;

//      /* d�ݼ�ѭ�������� */
//      sample--;
//    }

//    /*  �����º��״̬�����洢��pState������ */
//    *pState++ = Xn1;
//    *pState++ = Xn2;
//    *pState++ = Yn1;
//    *pState++ = Yn2;

//    /*��һ�׶δ����뻺���������������.     */
//    /*����numStages������������о͵ط���*/
//    pIn = pDst;

//    /* �������ָ�� */
//    pOut = pDst;

//    /* �ݼ�ѭ�������� */
//    stage--;

//  } while(stage > 0u);
//}

///*
//*����      :��ʼ���˲���
//*S        :ָ�򸡵�SOS�����ṹ��ʵ����
//*numStages:�˲����ж���SOS������
//*pCoeffs  :�˲�������ָ��,����������˳��洢
//*          {b10, b11, b12, a11, a12, b20, b21, b22, a21, a22, ...}
//*pState   :��ʷ״̬������ָ��
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
#include "hc32_ddl.h"

#define HASH_MSG_DIGEST_SIZE        (32u)
const static uint8_t pu8AesKey[] = "abcdefghijklmn;;";
const static uint8_t *pu8Plaintext1 = "123";//"123abcdefg456789";
const static uint8_t *pu8Plaintext2 = "knsjdviVFTGYHVAUB<>?^&*E$%^fnvcinj13216%^*((**%^%$<>?><<B";
static uint8_t u8SrcData1[] = "abcde";
static uint8_t u8SrcData2[] = \
"abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

static uint8_t u8SrcData3[] = \
"abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
!@#$%^&*( ()(*()*&&*^*& &^%^%^%dsf@#^%$#(*&^_)#<>?>?><>. ~;/':dd////ghe/\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
?></.,;;://}{[]}{///sdfas\"dfas;;;;;d\"\"\"\"\"}{|\\][?><\":56789ABCDEFGH\
abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789\
01234567890123456789";
static uint8_t u8HashMsgDigest1[HASH_MSG_DIGEST_SIZE];
static uint8_t u8HashMsgDigest2[HASH_MSG_DIGEST_SIZE];
static uint8_t u8HashMsgDigest3[HASH_MSG_DIGEST_SIZE];

static uint8_t u8Ciphertext[128];
static uint8_t u8Plaintext[128];

void User_CryptoEngine_Init(void)
{
    //AES_Init();
    HASH_Init();
}

void User_TestHashandAES(void)
{
    en_result_t Result_crypto;
		uint32_t u32CiphertextSize;
    memset(u8Ciphertext, 0u, sizeof(u8Ciphertext));
    printf("source Text:\r\n%s\r\n",pu8Plaintext2);
    AES_Encrypt(pu8Plaintext2, strlen((char *)pu8Plaintext2), pu8AesKey, u8Ciphertext);
    printf("Ciphertext:\r\n%s\r\n", u8Ciphertext);
    AES_Decrypt(u8Ciphertext, strlen((char *)u8Ciphertext), pu8AesKey, u8Plaintext);
    printf("Plaintext:\r\n%s\r\n",u8Plaintext);
    HASH_Start(pu8Plaintext2, strlen((char *)pu8Plaintext2), u8HashMsgDigest1, 10u);
    printf("Hash Source1:\r\n%s\r\n", u8SrcData1);
    Result_crypto = HASH_Start(u8SrcData1, 5, u8HashMsgDigest1, 10u);
    if(Ok == Result_crypto)
    {
        printf("Hash Disgest1:\r\n%s\r\n",u8HashMsgDigest1);
    }
    printf("Hash Source2:\r\n%s\r\n", u8SrcData2);
    HASH_Start(u8SrcData2, strlen((char *)u8SrcData2), u8HashMsgDigest2, 10u);
    printf("Hash Disgest2:\r\n %s\r\n",u8HashMsgDigest2);
//    HASH_Start(u8SrcData3, strlen((char *)u8SrcData3), u8HashMsgDigest3, 10u);   
}

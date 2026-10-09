#include "stm32f10x.h"
#include "ParamStorage.h"

static uint32_t ParamStorage_Crc32(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

uint8_t ParamStorage_Load(RobotParamBlock *params)
{
    const RobotParamBlock *stored;
    uint32_t expected_crc;

    if (params == 0)
    {
        return 0;
    }

    stored = (const RobotParamBlock *)PARAM_FLASH_ADDRESS;
    *params = *stored;

    if (params->magic != PARAM_MAGIC ||
        params->version != PARAM_VERSION ||
        params->size != sizeof(RobotParamBlock))
    {
        return 0;
    }

    expected_crc = ParamStorage_Crc32((const uint8_t *)params,
                                      (uint16_t)(sizeof(RobotParamBlock) - sizeof(uint32_t)));
    return (expected_crc == params->crc32);
}

uint8_t ParamStorage_Save(const RobotParamBlock *params)
{
    RobotParamBlock data;
    RobotParamBlock verify;
    const uint16_t *half_words;
    uint16_t count;
    uint16_t i;
    FLASH_Status status;

    if (params == 0)
    {
        return 0;
    }

    data = *params;
    data.magic = PARAM_MAGIC;
    data.version = PARAM_VERSION;
    data.size = sizeof(RobotParamBlock);
    data.crc32 = ParamStorage_Crc32((const uint8_t *)&data,
                                    (uint16_t)(sizeof(RobotParamBlock) - sizeof(uint32_t)));

    half_words = (const uint16_t *)&data;
    count = (uint16_t)(sizeof(RobotParamBlock) / sizeof(uint16_t));

    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    status = FLASH_ErasePage(PARAM_FLASH_ADDRESS);

    for (i = 0; i < count && status == FLASH_COMPLETE; i++)
    {
        status = FLASH_ProgramHalfWord(PARAM_FLASH_ADDRESS + (uint32_t)i * 2U,
                                       half_words[i]);
    }

    FLASH_Lock();
    __enable_irq();

    if (status != FLASH_COMPLETE)
    {
        return 0;
    }

    if (!ParamStorage_Load(&verify))
    {
        return 0;
    }
    return (verify.crc32 == data.crc32);
}

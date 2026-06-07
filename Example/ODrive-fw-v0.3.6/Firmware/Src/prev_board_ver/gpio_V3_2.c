/** 引脚配置为
        * 模拟输入
        * 数字输入
        * 数字输出
        * 事件输出
        * 外部中断
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* 使能GPIO端口时钟 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* 配置GPIO引脚输出电平 */
  HAL_GPIO_WritePin(GPIOC, M0_nCS_Pin|M1_nCS_Pin, GPIO_PIN_SET);

  /* 配置GPIO引脚输出电平 */
  HAL_GPIO_WritePin(GPIOC, M1_DC_CAL_Pin|M0_DC_CAL_Pin, GPIO_PIN_RESET);

  /* 配置GPIO引脚输出电平 */
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);

  /* 配置GPIO引脚: PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = M0_nCS_Pin|M1_nCS_Pin|M1_DC_CAL_Pin|M0_DC_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 配置GPIO引脚: PAPin PAPin */
  GPIO_InitStruct.Pin = GPIO_4_Pin|GPIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = GPIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_3_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = GPIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIO_1_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_GATE_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = M0_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M0_ENC_Z_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

  /* 配置GPIO引脚: PtPin */
  GPIO_InitStruct.Pin = M1_ENC_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(M1_ENC_Z_GPIO_Port, &GPIO_InitStruct);

  /* 外部中断初始化 */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}
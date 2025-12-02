#include "keypad.h"

#define ROW_NUM 4
#define COL_NUM 4

// GPIO ? ?? (main.h?? ??? ??? ???? ?)
// Row pins (???) - R1, R2, R3, R4
GPIO_TypeDef* rowPorts[ROW_NUM] = {GPIOA, GPIOA, GPIOB, GPIOB};
uint16_t rowPins[ROW_NUM] = {R1_Pin, R2_Pin, R3_Pin, R4_Pin};

// Column pins (???) - C1, C2, C3, C4
GPIO_TypeDef* colPorts[COL_NUM] = {GPIOA, GPIOA, GPIOA, GPIOA};
uint16_t colPins[COL_NUM] = {C1_Pin, C2_Pin, C3_Pin, C4_Pin};

// ? ??
char keys[ROW_NUM][COL_NUM] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

void keypad_init(void) {
    // ?? Row? HIGH? ?? (?? ??)
    for(int i = 0; i < ROW_NUM; i++) {
        HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
    }
}

char keypad_get_key(void) {
    static uint32_t lastKeyTime = 0;
    static char lastKey = 0;
    uint32_t currentTime = HAL_GetTick();

    // ????: 50ms ??? ?? ? ?? ??
    if(currentTime - lastKeyTime < 50) {
        return 0;
    }

    for(int row = 0; row < ROW_NUM; row++) {
        // ?? ?? LOW? ??
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_RESET);

        // ?? ?? (?? ???)
        HAL_Delay(1);

        // ? ?? ??
        for(int col = 0; col < COL_NUM; col++) {
            if(HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET) {
                // ?? ?? - ?? ??? ??? ??
                while(HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET) {
                    HAL_Delay(10);
                }

                // ?? ?? ?? HIGH? ??
                for(int i = 0; i < ROW_NUM; i++) {
                    HAL_GPIO_WritePin(rowPorts[i], rowPins[i], GPIO_PIN_SET);
                }

                lastKeyTime = HAL_GetTick();
                lastKey = keys[row][col];
                return lastKey;
            }
        }

        // ?? ?? ?? HIGH? ??
        HAL_GPIO_WritePin(rowPorts[row], rowPins[row], GPIO_PIN_SET);
    }

    return 0; // ?? ??? ??
}

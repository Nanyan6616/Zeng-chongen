#include <stdio.h>
enum MHz
{
    GPIO_Speed_2MHz,
    GPIO_Speed_10MHz,
    GPIO_Speed_50MHz
};
struct GPIO
{
    int GPIO_Speed;
    int num1;
    int num2;
};

void GPIO_StructureInit(struct GPIO* s)
{
   s->GPIO_Speed = GPIO_Speed_2MHz;
   printf("%d\n",s->GPIO_Speed);
}

int main()
{
    struct GPIO m;
    GPIO_StructureInit(&m);
    return 0;
}
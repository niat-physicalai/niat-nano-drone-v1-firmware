#include <stdio.h>
#include "units.h"

int main(void)
{
    if (NIAT_FRAME_FLU != 0u) {
        printf("FAIL: frame constant\n");
        return 1;
    }
    printf("PASS: units header\n");
    return 0;
}

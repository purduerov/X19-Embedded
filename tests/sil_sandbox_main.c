#include <stdio.h>

extern void app_main(void);

int main(void) {
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("[Host SIL Sandbox] Launching app_main()...\r\n");
    app_main();
    return 0;
}

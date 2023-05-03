#include <stdio.h>
#include <unistd.h>
#include "../include/ittnotify.h"

__itt_domain* d_app = NULL;
__itt_domain* d_main = NULL;
__itt_domain* d_func = NULL;

__itt_string_handle* handle_app  = NULL;
__itt_string_handle* handle_main = NULL;
__itt_string_handle* handle_func = NULL;

void func(void) {
    __itt_task_begin(d_func, __itt_null, __itt_null, handle_func);
    usleep(1000);
    __itt_task_end(d_func);
}

int main(void) {
    d_app = __itt_domain_create("App");
    d_main = __itt_domain_create("App.Main");
    d_func = __itt_domain_create("App.Main.Func");

    handle_app  = __itt_string_handle_create("app");
    handle_main = __itt_string_handle_create("main");
    handle_func = __itt_string_handle_create("func");

    __itt_task_begin(d_app, __itt_null, __itt_null, handle_app);

    __itt_task_begin(d_main, __itt_null, __itt_null, handle_main);
    for (int k=0; k<1000; k++) {
        func();
    }
    __itt_task_end(d_main);

    fprintf(stderr, "Done!\n");

    __itt_task_end(d_app);
    return 0;
}

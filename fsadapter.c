#include <compiler.h>
#include <dirent.h>
#include <sys/stat.h>

char *get_name(struct dirent *d) {
    return d->d_name;
}

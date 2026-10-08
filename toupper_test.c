

int toupper_obvious(int c) {
  return c >= 'a' && c <= 'z' ? (c ^ 0x20) : c;
}

int toupper_subtract(int c) {
  return (unsigned)c - 'a' < 26u ? (c ^ 0x20) : c;
}

int toupper_better(int c) {
    __asm {
            cmp  c,#'a' wc;
    if_nc   cmpr c,#'z' wc;
    if_nc   xor  c,#0x20;
    }
    return c;
}

int main() {
    toupper_obvious(_OUTA);
    toupper_obvious(_OUTA);
    toupper_obvious(_OUTA);
    toupper_subtract(_OUTA);
    toupper_subtract(_OUTA);
    toupper_subtract(_OUTA);
    toupper_better(_OUTA);
    toupper_better(_OUTA);
    toupper_better(_OUTA);
}
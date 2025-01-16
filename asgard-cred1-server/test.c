#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main() {
  char dashline[40];
  printf("strlen(dashline) = %ld\n", strlen(dashline));
  memset(dashline, '-', 40);
  printf("%s\n", dashline);
  printf("strlen(dashline) = %ld\n", strlen(dashline));
  exit(0);
}


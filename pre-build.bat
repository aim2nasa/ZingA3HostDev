del *.o  *.d *.elf *.img /S
cd ../../
git log --pretty=format:"#define GIT_INFO_PRESENT %%n static const char* GIT_INFO = \"Version Information=%%H\"; " -n 1 > gitcommit.h
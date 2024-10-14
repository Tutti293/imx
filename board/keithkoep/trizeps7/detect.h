#if 0
#define MXC_CPU_MX6Q            63
#define MXC_CPU_MX6DL           61
#define MXC_CPU_MX6SL           60
#define MXC_CPU_MX6SOLO         0x65 /* dummy ID */
#else
#include <asm/arch-imx/cpu.h>
#endif

# define mxc_cpu_type            __mxc_cpu_type
# define cpu_is_mx6q()          (mxc_cpu_type == MXC_CPU_MX6Q)
# define cpu_is_mx6dl()         (mxc_cpu_type == MXC_CPU_MX6DL)
# define cpu_is_mx6sl()         (mxc_cpu_type == MXC_CPU_MX6SL)

int verify_mx6q(void);
int verify_mx6dl(void);

extern unsigned int  __mxc_cpu_type;
void mx6_set_cpu_type(void);
unsigned long GetRAMSize(void);

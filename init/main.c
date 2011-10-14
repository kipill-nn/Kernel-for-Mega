/*
 *  linux/init/main.c
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  GK 2/5/95  -  Changed to support mounting root fs via NFS
 *  Added initrd & change_root: Werner Almesberger & Hans Lermen, Feb '96
 *  Moan early if gcc is old, avoiding bogus kernels - Paul Gortmaker, May '96
 *  Simplified starting of init:  Michael A. Griffith <grif@acm.org> 
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/utsname.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/initrd.h>
#include <linux/bootmem.h>
#include <linux/tty.h>
#include <linux/gfp.h>
#include <linux/percpu.h>
#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/kernel_stat.h>
#include <linux/start_kernel.h>
#include <linux/security.h>
#include <linux/smp.h>
#include <linux/workqueue.h>
#include <linux/profile.h>
#include <linux/rcupdate.h>
#include <linux/moduleparam.h>
#include <linux/kallsyms.h>
#include <linux/writeback.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/cgroup.h>
#include <linux/efi.h>
#include <linux/tick.h>
#include <linux/interrupt.h>
#include <linux/taskstats_kern.h>
#include <linux/delayacct.h>
#include <linux/unistd.h>
#include <linux/rmap.h>
#include <linux/mempolicy.h>
#include <linux/key.h>
#include <linux/buffer_head.h>
#include <linux/page_cgroup.h>
#include <linux/debug_locks.h>
#include <linux/debugobjects.h>
#include <linux/lockdep.h>
#include <linux/pid_namespace.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/idr.h>
#include <linux/ftrace.h>
#include <linux/async.h>
#include <trace/boot.h>

#include <asm/io.h>
#include <asm/bugs.h>
#include <asm/setup.h>
#include <asm/sections.h>
#include <asm/cacheflush.h>

#ifdef CONFIG_X86_LOCAL_APIC
#include <asm/smp.h>
#endif

#include <linux/fb.h>
static int kernel_init(void *);

extern void init_IRQ(void);
extern void fork_init(unsigned long);
extern void mca_init(void);
extern void sbus_init(void);
extern void prio_tree_init(void);
extern void radix_tree_init(void);
extern void free_initmem(void);
#ifdef	CONFIG_ACPI
extern void acpi_early_init(void);
#else
static inline void acpi_early_init(void) { }
#endif
#ifndef CONFIG_DEBUG_RODATA
static inline void mark_rodata_ro(void) { }
#endif

#ifdef CONFIG_TC
extern void tc_init(void);
#endif

enum system_states system_state __read_mostly;
EXPORT_SYMBOL(system_state);

/*
 * Boot command-line arguments
 */
#define MAX_INIT_ARGS CONFIG_INIT_ENV_ARG_LIMIT
#define MAX_INIT_ENVS CONFIG_INIT_ENV_ARG_LIMIT

extern void time_init(void);
/* Default late time init is NULL. archs can override this later. */
void (*__initdata late_time_init)(void);
extern void softirq_init(void);

/* Untouched command line saved by arch-specific code. */
char __initdata boot_command_line[COMMAND_LINE_SIZE];
/* Untouched saved command line (eg. for /proc) */
char *saved_command_line;
/* Command line for parameter parsing */
static char *static_command_line;

static char *execute_command;
static char *ramdisk_execute_command;

#ifdef CONFIG_SMP
/* Setup configured maximum number of CPUs to activate */
unsigned int __initdata setup_max_cpus = NR_CPUS;

/*
 * Setup routine for controlling SMP activation
 *
 * Command-line option of "nosmp" or "maxcpus=0" will disable SMP
 * activation entirely (the MPS table probe still happens, though).
 *
 * Command-line option of "maxcpus=<NUM>", where <NUM> is an integer
 * greater than 0, limits the maximum number of CPUs activated in
 * SMP mode to <NUM>.
 */
#ifndef CONFIG_X86_IO_APIC
static inline void disable_ioapic_setup(void) {};
#endif

static int __init nosmp(char *str)
{
	setup_max_cpus = 0;
	disable_ioapic_setup();
	return 0;
}

early_param("nosmp", nosmp);

static int __init maxcpus(char *str)
{
	get_option(&str, &setup_max_cpus);
	if (setup_max_cpus == 0)
		disable_ioapic_setup();

	return 0;
}

early_param("maxcpus", maxcpus);
#else
#define setup_max_cpus NR_CPUS
#endif

/*
 * If set, this is an indication to the drivers that reset the underlying
 * device before going ahead with the initialization otherwise driver might
 * rely on the BIOS and skip the reset operation.
 *
 * This is useful if kernel is booting in an unreliable environment.
 * For ex. kdump situaiton where previous kernel has crashed, BIOS has been
 * skipped and devices will be in unknown state.
 */
unsigned int reset_devices;
EXPORT_SYMBOL(reset_devices);

static int __init set_reset_devices(char *str)
{
	reset_devices = 1;
	return 1;
}

__setup("reset_devices", set_reset_devices);

static char * argv_init[MAX_INIT_ARGS+2] = { "init", NULL, };
char * envp_init[MAX_INIT_ENVS+2] = { "HOME=/", "TERM=linux", NULL, };
static const char *panic_later, *panic_param;

extern struct obs_kernel_param __setup_start[], __setup_end[];

static int __init obsolete_checksetup(char *line)
{
	struct obs_kernel_param *p;
	int had_early_param = 0;

	p = __setup_start;
	do {
		int n = strlen(p->str);
		if (!strncmp(line, p->str, n)) {
			if (p->early) {
				/* Already done in parse_early_param?
				 * (Needs exact match on param part).
				 * Keep iterating, as we can have early
				 * params and __setups of same names 8( */
				if (line[n] == '\0' || line[n] == '=')
					had_early_param = 1;
			} else if (!p->setup_func) {
				printk(KERN_WARNING "Parameter %s is obsolete,"
				       " ignored\n", p->str);
				return 1;
			} else if (p->setup_func(line + n))
				return 1;
		}
		p++;
	} while (p < __setup_end);

	return had_early_param;
}

/*
 * This should be approx 2 Bo*oMips to start (note initial shift), and will
 * still work even if initially too large, it will just take slightly longer
 */
unsigned long loops_per_jiffy = (1<<12);

EXPORT_SYMBOL(loops_per_jiffy);

static int __init debug_kernel(char *str)
{
	console_loglevel = 10;
	return 0;
}

static int __init quiet_kernel(char *str)
{
	console_loglevel = 4;
	return 0;
}

early_param("debug", debug_kernel);
early_param("quiet", quiet_kernel);

static int __init loglevel(char *str)
{
	get_option(&str, &console_loglevel);
	return 0;
}

early_param("loglevel", loglevel);

/*
 * Unknown boot options get handed to init, unless they look like
 * failed parameters
 */
static int __init unknown_bootoption(char *param, char *val)
{
	/* Change NUL term back to "=", to make "param" the whole string. */
	if (val) {
		/* param=val or param="val"? */
		if (val == param+strlen(param)+1)
			val[-1] = '=';
		else if (val == param+strlen(param)+2) {
			val[-2] = '=';
			memmove(val-1, val, strlen(val)+1);
			val--;
		} else
			BUG();
	}

	/* Handle obsolete-style parameters */
	if (obsolete_checksetup(param))
		return 0;

	/*
	 * Preemptive maintenance for "why didn't my misspelled command
	 * line work?"
	 */
	if (strchr(param, '.') && (!val || strchr(param, '.') < val)) {
		printk(KERN_ERR "Unknown boot option `%s': ignoring\n", param);
		return 0;
	}

	if (panic_later)
		return 0;

	if (val) {
		/* Environment option */
		unsigned int i;
		for (i = 0; envp_init[i]; i++) {
			if (i == MAX_INIT_ENVS) {
				panic_later = "Too many boot env vars at `%s'";
				panic_param = param;
			}
			if (!strncmp(param, envp_init[i], val - param))
				break;
		}
		envp_init[i] = param;
	} else {
		/* Command line option */
		unsigned int i;
		for (i = 0; argv_init[i]; i++) {
			if (i == MAX_INIT_ARGS) {
				panic_later = "Too many boot init vars at `%s'";
				panic_param = param;
			}
		}
		argv_init[i] = param;
	}
	return 0;
}

#ifdef CONFIG_DEBUG_PAGEALLOC
int __read_mostly debug_pagealloc_enabled = 0;
#endif

static int __init init_setup(char *str)
{
	unsigned int i;

	execute_command = str;
	/*
	 * In case LILO is going to boot us with default command line,
	 * it prepends "auto" before the whole cmdline which makes
	 * the shell think it should execute a script with such name.
	 * So we ignore all arguments entered _before_ init=... [MJ]
	 */
	for (i = 1; i < MAX_INIT_ARGS; i++)
		argv_init[i] = NULL;
	return 1;
}
__setup("init=", init_setup);

static int __init rdinit_setup(char *str)
{
	unsigned int i;

	ramdisk_execute_command = str;
	/* See "auto" comment in init_setup */
	for (i = 1; i < MAX_INIT_ARGS; i++)
		argv_init[i] = NULL;
	return 1;
}
__setup("rdinit=", rdinit_setup);

#ifndef CONFIG_SMP

#ifdef CONFIG_X86_LOCAL_APIC
static void __init smp_init(void)
{
	APIC_init_uniprocessor();
}
#else
#define smp_init()	do { } while (0)
#endif

static inline void setup_per_cpu_areas(void) { }
static inline void setup_nr_cpu_ids(void) { }
static inline void smp_prepare_cpus(unsigned int maxcpus) { }

#else

#if NR_CPUS > BITS_PER_LONG
cpumask_t cpu_mask_all __read_mostly = CPU_MASK_ALL;
EXPORT_SYMBOL(cpu_mask_all);
#endif

/* Setup number of possible processor ids */
int nr_cpu_ids __read_mostly = NR_CPUS;
EXPORT_SYMBOL(nr_cpu_ids);

/* An arch may set nr_cpu_ids earlier if needed, so this would be redundant */
static void __init setup_nr_cpu_ids(void)
{
	nr_cpu_ids = find_last_bit(cpumask_bits(cpu_possible_mask),NR_CPUS) + 1;
}

#ifndef CONFIG_HAVE_SETUP_PER_CPU_AREA
unsigned long __per_cpu_offset[NR_CPUS] __read_mostly;

EXPORT_SYMBOL(__per_cpu_offset);

static void __init setup_per_cpu_areas(void)
{
	unsigned long size, i;
	char *ptr;
	unsigned long nr_possible_cpus = num_possible_cpus();

	/* Copy section for each CPU (we discard the original) */
	size = ALIGN(PERCPU_ENOUGH_ROOM, PAGE_SIZE);
	ptr = alloc_bootmem_pages(size * nr_possible_cpus);

	for_each_possible_cpu(i) {
		__per_cpu_offset[i] = ptr - __per_cpu_start;
		memcpy(ptr, __per_cpu_start, __per_cpu_end - __per_cpu_start);
		ptr += size;
	}
}
#endif /* CONFIG_HAVE_SETUP_PER_CPU_AREA */

/* Called by boot processor to activate the rest. */
static void __init smp_init(void)
{
	unsigned int cpu;

	/*
	 * Set up the current CPU as possible to migrate to.
	 * The other ones will be done by cpu_up/cpu_down()
	 */
	cpu = smp_processor_id();
	cpu_set(cpu, cpu_active_map);

	/* FIXME: This should be done in userspace --RR */
	for_each_present_cpu(cpu) {
		if (num_online_cpus() >= setup_max_cpus)
			break;
		if (!cpu_online(cpu))
			cpu_up(cpu);
	}

	/* Any cleanup work */
	printk(KERN_INFO "Brought up %ld CPUs\n", (long)num_online_cpus());
	smp_cpus_done(setup_max_cpus);
}

#endif

/*
 * We need to store the untouched command line for future reference.
 * We also need to store the touched command line since the parameter
 * parsing is performed in place, and we should allow a component to
 * store reference of name/value for future reference.
 */
static void __init setup_command_line(char *command_line)
{
	saved_command_line = alloc_bootmem(strlen (boot_command_line)+1);
	static_command_line = alloc_bootmem(strlen (command_line)+1);
	strcpy (saved_command_line, boot_command_line);
	strcpy (static_command_line, command_line);
}

/*
 * We need to finalize in a non-__init function or else race conditions
 * between the root thread and the init thread may cause start_kernel to
 * be reaped by free_initmem before the root thread has proceeded to
 * cpu_idle.
 *
 * gcc-3.4 accidentally inlines this function, so use noinline.
 */

static noinline void __init_refok rest_init(void)
	__releases(kernel_lock)
{
	int pid;

	kernel_thread(kernel_init, NULL, CLONE_FS | CLONE_SIGHAND);
	numa_default_policy();
	pid = kernel_thread(kthreadd, NULL, CLONE_FS | CLONE_FILES);
	kthreadd_task = find_task_by_pid_ns(pid, &init_pid_ns);
	unlock_kernel();

	/*
	 * The boot idle thread must execute schedule()
	 * at least once to get things moving:
	 */
	init_idle_bootup_task(current);
	rcu_scheduler_starting();
	preempt_enable_no_resched();
	schedule();
	preempt_disable();

	/* Call into cpu_idle with preempt disabled */
	cpu_idle();
}

/* Check for early params. */
static int __init do_early_param(char *param, char *val)
{
	struct obs_kernel_param *p;

	for (p = __setup_start; p < __setup_end; p++) {
		if ((p->early && strcmp(param, p->str) == 0) ||
		    (strcmp(param, "console") == 0 &&
		     strcmp(p->str, "earlycon") == 0)
		) {
			if (p->setup_func(val) != 0)
				printk(KERN_WARNING
				       "Malformed early option '%s'\n", param);
		}
	}
	/* We accept everything at this stage. */
	return 0;
}

/* Arch code calls this early on, or if not, just before other parsing. */
void __init parse_early_param(void)
{
	static __initdata int done = 0;
	static __initdata char tmp_cmdline[COMMAND_LINE_SIZE];

	if (done)
		return;

	/* All fall through to do_early_param. */
	strlcpy(tmp_cmdline, boot_command_line, COMMAND_LINE_SIZE);
	parse_args("early options", tmp_cmdline, NULL, 0, do_early_param);
	done = 1;
}

/*
 *	Activate the first processor.
 */

static void __init boot_cpu_init(void)
{
	int cpu = smp_processor_id();
	/* Mark the boot cpu "present", "online" etc for SMP and UP case */
	set_cpu_online(cpu, true);
	set_cpu_present(cpu, true);
	set_cpu_possible(cpu, true);
}

void __init __weak smp_setup_processor_id(void)
{
}

void __init __weak thread_info_cache_init(void)
{
}

/*
 * Perform a memory test. A more complete alternative test can be
 * configured using CONFIG_SYS_ALT_MEMTEST. The complete test loops until
 * interrupted by ctrl-c or by a failure of one of the sub-tests.
 */
int do_mem_mtest ()
{
	ulong	*addr, *start, *end;
	ulong	val;
	ulong	readback;
	ulong	errs = 0;
	int iterations = 1;
	int iteration_limit;

#if defined(CONFIG_SYS_ALT_MEMTEST)
	ulong	len;
	ulong	offset;
	ulong	test_offset;
	ulong	pattern;
	ulong	temp;
	ulong	anti_pattern;
	ulong	num_words;
#if defined(CONFIG_SYS_MEMTEST_SCRATCH)
	ulong *dummy = (ulong*)CONFIG_SYS_MEMTEST_SCRATCH;
#else
	ulong *dummy = 0;	/* yes, this is address 0x0, not NULL */
#endif
	int	j;

	static const ulong bitpattern[] = {
		0x00000001,	/* single bit */
		0x00000003,	/* two adjacent bits */
		0x00000007,	/* three adjacent bits */
		0x0000000F,	/* four adjacent bits */
		0x00000005,	/* two non-adjacent bits */
		0x00000015,	/* three non-adjacent bits */
		0x00000055,	/* four non-adjacent bits */
		0xaaaaaaaa,	/* alternating 1/0 */
	};
#else
	ulong	incr;
	ulong	pattern;
#endif

		start = (ulong *)0xf9600000;

		end = (ulong *)0x000000ff;

		pattern = 0;

		iteration_limit = 0;

#if defined(CONFIG_SYS_ALT_MEMTEST)
	printk ("Testing %08x ... %08x:\n", (uint)start, (uint)end);
	PRINTF("%s:%d: start 0x%p end 0x%p\n",
		__FUNCTION__, __LINE__, start, end);

	for (;;) {


		if (iteration_limit && iterations > iteration_limit) {
			printk("Tested %d iteration(s) with %lu errors.\n",
				iterations-1, errs);
			return errs != 0;
		}

		printk("Iteration: %6d\r", iterations);
		PRINTF("\n");
		iterations++;

		/*
		 * Data line test: write a pattern to the first
		 * location, write the 1's complement to a 'parking'
		 * address (changes the state of the data bus so a
		 * floating bus doen't give a false OK), and then
		 * read the value back. Note that we read it back
		 * into a variable because the next time we read it,
		 * it might be right (been there, tough to explain to
		 * the quality guys why it prints a failure when the
		 * "is" and "should be" are obviously the same in the
		 * error message).
		 *
		 * Rather than exhaustively testing, we test some
		 * patterns by shifting '1' bits through a field of
		 * '0's and '0' bits through a field of '1's (i.e.
		 * pattern and ~pattern).
		 */
		addr = start;
		for (j = 0; j < sizeof(bitpattern)/sizeof(bitpattern[0]); j++) {
		    val = bitpattern[j];
		    for(; val != 0; val <<= 1) {
			*addr  = val;
			*dummy  = ~val; /* clear the test data off of the bus */
			readback = *addr;
			if(readback != val) {
			    printk ("FAILURE (data line): "
				"expected %08lx, actual %08lx\n",
					  val, readback);
			    errs++;
			    }
			}
			*addr  = ~val;
			*dummy  = val;
			readback = *addr;
			if(readback != ~val) {
			    printk ("FAILURE (data line): "
				"Is %08lx, should be %08lx\n",
					readback, ~val);
			    errs++;
			    }
			}
		    }
		}

		/*
		 * Based on code whose Original Author and Copyright
		 * information follows: Copyright (c) 1998 by Michael
		 * Barr. This software is placed into the public
		 * domain and may be used for any purpose. However,
		 * this notice must not be changed or removed and no
		 * warranty is either expressed or implied by its
		 * publication or distribution.
		 */

		/*
		 * Address line test
		 *
		 * Description: Test the address bus wiring in a
		 *              memory region by performing a walking
		 *              1's test on the relevant bits of the
		 *              address and checking for aliasing.
		 *              This test will find single-bit
		 *              address failures such as stuck -high,
		 *              stuck-low, and shorted pins. The base
		 *              address and size of the region are
		 *              selected by the caller.
		 *
		 * Notes:	For best results, the selected base
		 *              address should have enough LSB 0's to
		 *              guarantee single address bit changes.
		 *              For example, to test a 64-Kbyte
		 *              region, select a base address on a
		 *              64-Kbyte boundary. Also, select the
		 *              region size as a power-of-two if at
		 *              all possible.
		 *
		 * Returns:     0 if the test succeeds, 1 if the test fails.
		 */
		len = ((ulong)end - (ulong)start)/sizeof(ulong);
		pattern = (ulong) 0xaaaaaaaa;
		anti_pattern = (ulong) 0x55555555;

		PRINTF("%s:%d: length = 0x%.8lx\n",
			__FUNCTION__, __LINE__,
			len);
		/*
		 * Write the default pattern at each of the
		 * power-of-two offsets.
		 */
		for (offset = 1; offset < len; offset <<= 1) {
			start[offset] = pattern;
		}

		/*
		 * Check for address bits stuck high.
		 */
		test_offset = 0;
		start[test_offset] = anti_pattern;

		for (offset = 1; offset < len; offset <<= 1) {
		    temp = start[offset];
		    if (temp != pattern) {
			printk ("\nFAILURE: Address bit stuck high @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx\n",
				(ulong)&start[offset], pattern, temp);
			errs++;
		    }
		}
		start[test_offset] = pattern;

		/*
		 * Check for addr bits stuck low or shorted.
		 */
		for (test_offset = 1; test_offset < len; test_offset <<= 1) {
		    start[test_offset] = anti_pattern;

		    for (offset = 1; offset < len; offset <<= 1) {
			temp = start[offset];
			if ((temp != pattern) && (offset != test_offset)) {
			    printk ("\nFAILURE: Address bit stuck low or shorted @"
				" 0x%.8lx: expected 0x%.8lx, actual 0x%.8lx\n",
				(ulong)&start[offset], pattern, temp);
			    errs++;
			}
		    }
		    start[test_offset] = pattern;
		}

		/*
		 * Description: Test the integrity of a physical
		 *		memory device by performing an
		 *		increment/decrement test over the
		 *		entire region. In the process every
		 *		storage bit in the device is tested
		 *		as a zero and a one. The base address
		 *		and the size of the region are
		 *		selected by the caller.
		 *
		 * Returns:     0 if the test succeeds, 1 if the test fails.
		 */
		num_words = ((ulong)end - (ulong)start)/sizeof(ulong) + 1;

		/*
		 * Fill memory with a known pattern.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
			start[offset] = pattern;
		}

		/*
		 * Check each location and invert it for the second pass.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		    temp = start[offset];
		    if (temp != pattern) {
			printk ("\nFAILURE (read/write) @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				(ulong)&start[offset], pattern, temp);
			errs++;
		    }

		    anti_pattern = ~pattern;
		    start[offset] = anti_pattern;
		}

		/*
		 * Check each location for the inverted pattern and zero it.
		 */
		for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++) {
		    anti_pattern = ~pattern;
		    temp = start[offset];
		    if (temp != anti_pattern) {
			printk ("\nFAILURE (read/write): @ 0x%.8lx:"
				" expected 0x%.8lx, actual 0x%.8lx)\n",
				(ulong)&start[offset], anti_pattern, temp);
			errs++;
		    }
		    start[offset] = 0;
		}
	}

#else /* The original, quickie test */
	incr = 1;
	for (;;) {

		if (iteration_limit && iterations > iteration_limit) {
			printk("Tested %d iteration(s) with %lu errors.\n",
				iterations-1, errs);
			return errs != 0;
		}
		++iterations;

		printk ("\rPattern %08lX  Writing..."
			"%12s"
			"\b\b\b\b\b\b\b\b\b\b",
			pattern, "");

		for (addr=start,val=pattern; addr<end; addr++) {
			*addr = val;
			val  += incr;
		}

		printk ("Reading...");

		for (addr=start,val=pattern; addr<end; addr++) {
			readback = *addr;
			if (readback != val) {
				printk ("\nMem error @ 0x%08X: "
					"found %08lX, expected %08lX\n",
					(uint)addr, readback, val);
				errs++;
			}
			val += incr;
		}

		/*
		 * Flip the pattern each time to make lots of zeros and
		 * then, the next time, lots of ones.  We decrement
		 * the "negative" patterns and increment the "positive"
		 * patterns to preserve this feature.
		 */
		if(pattern & 0x80000000) {
			pattern = -pattern;	/* complement & increment */
		}
		else {
			pattern = ~pattern;
		}
		incr = -incr;
	}
#endif
	return 0;	/* not reached */
}

asmlinkage void __init start_kernel(void)
{
	char * command_line;
	extern struct kernel_param __start___param[], __stop___param[];

	smp_setup_processor_id();

	/*
	 * Need to run as early as possible, to initialize the
	 * lockdep hash:
	 */
        printk("setup processor id\n");
	lockdep_init();
        printk("lockdep init\n");
	debug_objects_early_init();
	cgroup_init_early();

	local_irq_disable();
	early_boot_irqs_off();
	early_init_irq_lock_class();

/*
 * Interrupts are still disabled. Do necessary setups, then
 * enable them
 */
	lock_kernel();
	tick_init();
	boot_cpu_init();
	page_address_init();
        printk("page address init\n");
	printk(KERN_NOTICE);
	printk(linux_banner);
	setup_arch(&command_line);
        printk("setup arch\n");
	mm_init_owner(&init_mm, &init_task);
        printk("mm init owner\n");
	setup_command_line(command_line);
//        printk("\n\nsetup command line\n\n");
	setup_per_cpu_areas();
//        printk("\n\nsetup per cpu areas\n\n");
	setup_nr_cpu_ids();
//        printk("\n\nsetup nr cpu ids\n\n");
	smp_prepare_boot_cpu();	/* arch-specific boot-cpu hooks */

	/*
	 * Set up the scheduler prior starting any interrupts (such as the
	 * timer interrupt). Full topology setup happens at smp_init()
	 * time - but meanwhile we still have a functioning scheduler.
	 */
	sched_init();
//        printk("\n\nsched init\n\n");
	/*
	 * Disable preemption - early bootup scheduling is extremely
	 * fragile until we cpu_idle() for the first time.
	 */
	preempt_disable();
//        printk("\n\npreempt disable\n\n");
	build_all_zonelists();
//       printk("\n\nbuild all zonelists\n\n");
	page_alloc_init();
//        printk("\n\npage alloc init\n\n");
	printk(KERN_NOTICE "Kernel command line: %s\n", boot_command_line);
	parse_early_param();
	parse_args("Booting kernel", static_command_line, __start___param,
		   __stop___param - __start___param,
		   &unknown_bootoption);
	if (!irqs_disabled()) {
		printk(KERN_WARNING "start_kernel(): bug: interrupts were "
				"enabled *very* early, fixing it\n");
		local_irq_disable();
	}
	sort_main_extable();
//        printk("\n\nsort main extable\n\n");
	trap_init();
//        printk("\n\n   trap init\n\n");
	rcu_init();
//        printk("\n\n   rcu init\n\n");
	/* init some links before init_ISA_irqs() */
	early_irq_init();
//        printk("\n\n   early irq init\n\n");
	init_IRQ();
//        printk("\n\n   init IRQ\n\n");
	pidhash_init();
//        printk("\n\n   pidhash init\n\n");
	init_timers();
//        printk("\n\n   init timers\n\n");
	hrtimers_init();
//        printk("\n\n   hrtimers init\n\n");
	softirq_init();
//        printk("\n\n   softirq\n\n");
	timekeeping_init();
	time_init();
//        printk("\n\n   Time init\n\n");
	sched_clock_init();
	profile_init();
//        printk("\n\n   profile init\n\n");
	if (!irqs_disabled())
		printk(KERN_CRIT "start_kernel(): bug: interrupts were "
				 "enabled early\n");
	early_boot_irqs_on();
//        printk("\n\nearly boot irqs on\n\n");
	local_irq_enable();
//        printk("\n\nlocal irq enable\n\n");

	/*
	 * HACK ALERT! This is early. We're enabling the console before
	 * we've done PCI setups etc, and console_init() must be aware of
	 * this. But we do want output early, in case something goes wrong.
	 */
	console_init();
//        printk("\n\nconsole init\n\n");
	if (panic_later)
		panic(panic_later, panic_param);
        printk("console init\n");

	lockdep_info();

	/*
	 * Need to run this when irqs are enabled, because it wants
	 * to self-test [hard/soft]-irqs on/off lock inversion bugs
	 * too:
	 */
	locking_selftest();

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start && !initrd_below_start_ok &&
	    page_to_pfn(virt_to_page((void *)initrd_start)) < min_low_pfn) {
		printk(KERN_CRIT "initrd overwritten (0x%08lx < 0x%08lx) - "
		    "disabling it.\n",
		    page_to_pfn(virt_to_page((void *)initrd_start)),
		    min_low_pfn);
		initrd_start = 0;
	}
#endif
	vmalloc_init();
//        printk("\n\nmain.c: vmalloc init\n\n");
	vfs_caches_init_early();
//        printk("\n\nmain.c: vfs caches init early\n\n");
	cpuset_init_early();
//        printk("\n\nmain.c: cpuset init early\n\n");
	page_cgroup_init();
//        printk("page cgroup init\n");
	mem_init();
//        printk("mem init\n");
	enable_debug_pagealloc();
//        printk("\n\nmain.c: enable debug pagealloc\n\n");
	cpu_hotplug_init();
//        printk("\n\nmain.c: cpu hotplug init\n\n");
	kmem_cache_init();
//        printk("kmem cache init\n");
	debug_objects_mem_init();
//        printk("\n\nmain.c: debug objects mem init\n\n");
	idr_init_cache();
//        printk("\n\nmain.c: idr init cache\n\n");
	setup_per_cpu_pageset();
//        printk("\n\nmain.c: setup per cpu pageset\n\n");
	numa_policy_init();
//        printk("\n\nmain.c: numa policy init\n\n");
	if (late_time_init)
		late_time_init();
	calibrate_delay();
//        printk("\n\nmain.c: calibrate delay\n\n");
	pidmap_init();
//        printk("\n\nmain.c: pidmap init\n\n");
	pgtable_cache_init();
//        printk("\n\nmain.c: pgtable cache init\n\n");
	prio_tree_init();
//        printk("\n\nmain.c: prio tree init\n\n");
	anon_vma_init();
//        printk("\n\nmain.c: anon vma init\n\n");
#ifdef CONFIG_X86
	if (efi_enabled)
		efi_enter_virtual_mode();
#endif
	thread_info_cache_init();
//        printk("\n\nmain.c: thread info cache init\n\n");
	cred_init();
//        printk("\n\nmain.c: cred init\n\n");
	fork_init(num_physpages);
//        printk("\n\nmain.c: fork init\n\n");
	proc_caches_init();
//        printk("\n\nmain.c: proc caches init\n\n");
	buffer_init();
//        printk("\n\nmain.c: buffer init\n\n");
	key_init();
//        printk("\n\nmain.c: key init\n\n");
	security_init();
//        printk("\n\nmain.c: security init\n\n");
	vfs_caches_init(num_physpages);
//        printk("\n\nmain.c: vfs caches init\n\n");
	radix_tree_init();
//        printk("\n\nmain.c: radix tree init\n\n");
	signals_init();
//        printk("\n\nmain.c: signals init\n\n");
	/* rootfs populating might need page-writeback */
	page_writeback_init();
        printk("\n\nmain.c: page writeback init\n\n");
#ifdef CONFIG_PROC_FS
	proc_root_init();
#endif
	cgroup_init();
//        printk("\n\nmain.c: cgroup init\n\n");
	cpuset_init();
//        printk("\n\nmain.c: cpuset init\n\n");
	taskstats_init_early();
//        printk("\n\nmain.c: taskstats init early\n\n");
	delayacct_init();
//        printk("\n\nmain.c: delayacct init\n\n");
	check_bugs();
//        printk("\n\nmain.c: check bugs\n\n");
	acpi_early_init(); /* before LAPIC and SMP init */
//        printk("\n\nmain.c: acpi early init\n\n");
	ftrace_init();        
//        printk("\n\nmain.c: ftrace init\n\n");
    //do_mem_mtest();
	/* Do the rest non-__init'ed, we're now alive */
	rest_init();
//        printk("\n\nmain.c: rest init\n\n");
}

int initcall_debug = 0;
core_param(initcall_debug, initcall_debug, bool, 0644);

int do_one_initcall(initcall_t fn)
{
	int count = preempt_count();
	ktime_t calltime, delta, rettime;
	char msgbuf[64];
	struct boot_trace_call call;
	struct boot_trace_ret ret;

	if (initcall_debug) {
		call.caller = task_pid_nr(current);
		printk("calling  %pF @ %i\n", fn, call.caller);
		calltime = ktime_get();
		trace_boot_call(&call, fn);
		enable_boot_trace();
	}

	ret.result = fn();

	if (initcall_debug) {
		disable_boot_trace();
		rettime = ktime_get();
		delta = ktime_sub(rettime, calltime);
		ret.duration = (unsigned long long) ktime_to_ns(delta) >> 10;
		trace_boot_ret(&ret, fn);
		printk("initcall %pF returned %d after %Ld usecs\n", fn,
			ret.result, ret.duration);
	}

	msgbuf[0] = 0;

	if (ret.result && ret.result != -ENODEV && initcall_debug)
		sprintf(msgbuf, "error code %d ", ret.result);

	if (preempt_count() != count) {
		strlcat(msgbuf, "preemption imbalance ", sizeof(msgbuf));
		preempt_count() = count;
	}
	if (irqs_disabled()) {
		strlcat(msgbuf, "disabled interrupts ", sizeof(msgbuf));
		local_irq_enable();
	}
	if (msgbuf[0]) {
		printk("initcall %pF returned with %s\n", fn, msgbuf);
	}

	return ret.result;
}


extern initcall_t __initcall_start[], __initcall_end[], __early_initcall_end[];

static void __init do_initcalls(void)
{
	initcall_t *call;

	for (call = __early_initcall_end; call < __initcall_end; call++)
		do_one_initcall(*call);

	/* Make sure there is no pending stuff from the initcall sequence */
	flush_scheduled_work();
}

/*
 * Ok, the machine is now initialized. None of the devices
 * have been touched yet, but the CPU subsystem is up and
 * running, and memory and process management works.
 *
 * Now we can finally start doing some real work..
 */
static void __init do_basic_setup(void)
{
	rcu_init_sched(); /* needed by module_init stage. */
//        printk("\n\ndo_basic_setup(): rcu_init_sched\n\n");
	init_workqueues();
//        printk("\n\ndo_basic_setup(): init_workqueues\n\n");
	usermodehelper_init();
//        printk("\n\ndo_basic_setup(): usermodehelper\n\n");
	driver_init();
//        printk("\n\ndo_basic_setup(): driver_init\n\n");
	init_irq_proc();
//        printk("\n\ndo_basic_setup(): init_irq_proc\n\n");
	do_initcalls();
//        printk("\n\ndo_basic_setup(): do_initcalls\n\n");
}

static void __init do_pre_smp_initcalls(void)
{
	initcall_t *call;

	for (call = __initcall_start; call < __early_initcall_end; call++)
		do_one_initcall(*call);
}

static void run_init_process(char *init_filename)
{
	argv_init[0] = init_filename;
	kernel_execve(init_filename, argv_init, envp_init);
}

//#define INIT_IMAGE_FILE "/logo.rle"
//extern int load_565rle_image(char *filename);

/* This is a non __init function. Force it to be noinline otherwise gcc
 * makes it inline to init() and it becomes part of init.text section
 */
static noinline int init_post(void)
{
    int fd, fd_fb;
    unsigned count;
    unsigned short *data;

	/* need to finish all async __init code before freeing the memory */
	async_synchronize_full();
        printk("\n\nmain.c: asunc_synchronize_full\n\n");
	free_initmem();
        printk("\n\nmain.c: free_initmem\n\n");
	unlock_kernel();
        printk("\n\nmain.c: unlock_kernel\n\n");
	mark_rodata_ro();
        printk("\n\nmain.c: mark_rodata_ro\n\n");
	system_state = SYSTEM_RUNNING;
        printk("\n\nmain.c: system_state\n\n");
	numa_default_policy();
        printk("\n\nmain.c: numa_default_policy\n\n");
	if (sys_open((const char __user *) "/dev/console", O_RDWR, 0) < 0)
		printk(KERN_WARNING "Warning: unable to open an initial console.\n");

//    if(!load_565rle_image(INIT_IMAGE_FILE)) {
//        struct fb_info *info = registered_fb[0];
//        fb_pan_display(info, &info->var);
//    }
//#ifdef INIT_IMAGE_FILE
//    {
//        fd_fb = sys_open((const char __user *) "/dev/fb0", O_RDWR, 0);
//        if (fd_fb >= 0) {
//            fd = sys_open(INIT_IMAGE_FILE, O_RDONLY, 0);
//            if (fd >= 0) {
//                count = (unsigned)sys_lseek(fd, (off_t)0, 2);
//                if (count != 0) {
//                    sys_lseek(fd, (off_t)0, 0);
//                    data = kmalloc(count, GFP_KERNEL);
//                    sys_read(fd, (char *)data, count);
//                    sys_write(fd_fb, (char *)data, count);
//
//                    struct fb_info *info = registered_fb[0];
//                    fb_pan_display(info, &info->var);
//                }
//            }
//        }
//    }
//#endif

	(void) sys_dup(0);
	(void) sys_dup(0);

	current->signal->flags |= SIGNAL_UNKILLABLE;

        printk("\n\nmain.c: ramdisk_exe_cmd=%s\n\n", ramdisk_execute_command);
	if (ramdisk_execute_command) {
		run_init_process(ramdisk_execute_command);
		printk(KERN_WARNING "Failed to execute %s\n",
				ramdisk_execute_command);
	}

	/*
	 * We try each of these until one succeeds.
	 *
	 * The Bourne shell can be used instead of init if we are
	 * trying to recover a really broken machine.
	 */
	if (execute_command) {
		run_init_process(execute_command);
		printk(KERN_WARNING "Failed to execute %s.  Attempting "
					"defaults...\n", execute_command);
	}
	run_init_process("/sbin/init");
	run_init_process("/etc/init");
	run_init_process("/bin/init");
	run_init_process("/bin/sh");

	panic("No init found.  Try passing init= option to kernel.");
}

static int __init kernel_init(void * unused)
{
	lock_kernel();
	/*
	 * init can run on any cpu.
	 */
	set_cpus_allowed_ptr(current, CPU_MASK_ALL_PTR);
	/*
	 * Tell the world that we're going to be the grim
	 * reaper of innocent orphaned children.
	 *
	 * We don't want people to have to make incorrect
	 * assumptions about where in the task array this
	 * can be found.
	 */
	init_pid_ns.child_reaper = current;
//        printk("\n\nmain.c: init_pid_ns.child_reaper\n\n");
	cad_pid = task_pid(current);
//        printk("\n\nmain.c: cad_pid\n\n");
	smp_prepare_cpus(setup_max_cpus);
//        printk("\n\nmain.c: smp_prepare_cpus\n\n");
	do_pre_smp_initcalls();
//        printk("\n\nmain.c: do_pre_smp_initcalls\n\n");
	start_boot_trace();
//        printk("\n\nmain.c: start_boot_trace\n\n");
	smp_init();
//        printk("\n\nmain.c: smp_init\n\n");
	sched_init_smp();
//        printk("\n\nmain.c: sched_init_smp\n\n");
	cpuset_init_smp();
//        printk("\n\nmain.c: cpuset_init_smp\n\n");
	do_basic_setup();
//        printk("\n\nmain.c: do_basic_setup\n\n");
	/*
	 * check if there is an early userspace init.  If yes, let it do all
	 * the work
	 */

	if (!ramdisk_execute_command)
		ramdisk_execute_command = "/init";

	if (sys_access((const char __user *) ramdisk_execute_command, 0) != 0) {
		ramdisk_execute_command = NULL;
		prepare_namespace();
	}

	/*
	 * Ok, we have completed the initial bootup, and
	 * we're essentially up and running. Get rid of the
	 * initmem segments and start the user-mode stuff..
	 */
        printk("\n\nmain.c: Ok, we have completed the initial bootup\n\n");
	init_post();
        printk("\n\nmain.c: init_post\n\n");
	return 0;
}

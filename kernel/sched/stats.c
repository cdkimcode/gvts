
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>

#include "sched.h"

/*
 * bump this up when changing the output format or the meaning of an existing
 * format, so that tools can adapt (or abort)
 */
#define SCHEDSTAT_VERSION 16

static int show_schedstat(struct seq_file *seq, void *v)
{
	int cpu;

	if (v == (void *)1) {
		seq_printf(seq, "version %d\n", SCHEDSTAT_VERSION);
		seq_printf(seq, "timestamp %lu\n", jiffies);
	} else {
		struct rq *rq;
#ifdef CONFIG_SMP
		struct sched_domain *sd;
		int dcount = 0;
#endif
#ifdef CONFIG_GVFS_STATS
		enum cpu_idle_type itype;
#endif

		cpu = (unsigned long)(v - 2);
		rq = cpu_rq(cpu);

		/* runqueue-specific stats */
		seq_printf(seq,
		    "cpu%d %u 0 %u %u %u %u %llu %llu %lu",
		    cpu, rq->yld_count,
		    rq->sched_count, rq->sched_goidle,
		    rq->ttwu_count, rq->ttwu_local,
		    rq->rq_cpu_time,
		    rq->rq_sched_info.run_delay, rq->rq_sched_info.pcount);
#ifdef CONFIG_GVFS_STATS
		seq_printf(seq, " %u %u %u", 
			rq->nr_idle_balance_works, 
			rq->nr_running, 
			rq->cfs.h_nr_running
			);
		seq_printf(seq, " %u %u %u", 
			rq->select_fail, 
			rq->select_idle, 
			rq->select_busy
			);
		for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
				itype++) {
			seq_printf(seq, " %u %u", 
				rq->tvb_count[itype], 
				rq->tvb_fast_path[itype]
				);
		}
		seq_printf(seq, " %llu %llu",
					rq->cfs.target_vruntime,
#ifdef CONFIG_GVFS_REAL_MIN_VRUNTIME
					rq->cfs.real_min_vruntime
#else	
					rq->cfs.min_vruntime
#endif
					);
		seq_printf(seq, " %ld %lld %u %u %u %u %u %u %u %u %u\n",
					rq->cfs.lagged_weight, rq->cfs.lagged,
					rq->largest_idle_min_vruntime_racing,
					rq->satb_cond, /* Souce Activated Target-vruntime Balance */
					rq->satb_try,
					rq->satb_run,
					rq->satb_count,
					rq->satb_pushed,
					rq->get_traverse_rq_count,
					rq->get_traverse_child_count,
					rq->iterate_thrott_q
					);
#endif /* CONFIG_GVFS_STATS */

#ifdef CONFIG_SMP
		/* domain-specific stats */
		rcu_read_lock();
		for_each_domain(cpu, sd) {
			enum cpu_idle_type itype;

			seq_printf(seq, "domain%d %*pb", dcount++,
				   cpumask_pr_args(sched_domain_span(sd)));
			for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
					itype++) {
				seq_printf(seq, " %u %u %u %u %u %u %u %u",
				    sd->lb_count[itype],
				    sd->lb_balanced[itype],
				    sd->lb_failed[itype],
				    sd->lb_imbalance[itype],
				    sd->lb_gained[itype],
				    sd->lb_hot_gained[itype],
				    sd->lb_nobusyq[itype],
				    sd->lb_nobusyg[itype]);
			}
			seq_printf(seq,
				   " %u %u %u %u %u %u %u %u %u %u %u %u",
			    sd->alb_count, sd->alb_failed, sd->alb_pushed,
			    sd->sbe_count, sd->sbe_balanced, sd->sbe_pushed,
			    sd->sbf_count, sd->sbf_balanced, sd->sbf_pushed,
			    sd->ttwu_wake_remote, sd->ttwu_move_affine,
			    sd->ttwu_move_balance);
#ifdef CONFIG_GVFS_STATS
			seq_printf(seq,
				   " %u %u %u %u",
			    sd->atb_count, sd->atb_failed, sd->atb_pushed, sd->atb_pushed_under);
			for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
					itype++) {
				seq_printf(seq, " %u %u %u %u %u",
				    sd->tb_count[itype],
				    sd->tb_gained[itype],
				    sd->tb_nolaggedgroup[itype],
				    sd->tb_nolaggedcpu[itype],
				    sd->tb_all_pinned_but_running[itype]
					);
			}
			for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
					itype++) {
				seq_printf(seq, " %u %u %u %u %u %u %u %u %u",
				    sd->tvb_count[itype],
				    sd->tvb_not_reach[itype],
				    sd->tvb_idle_continue[itype],
				    sd->tvb_stay[itype],
				    sd->tvb_not_update[itype],
				    sd->tvb_update_target[itype],
				    sd->tvb_pull_count[itype],
				    sd->tvb_pull_gained[itype],
				    sd->tvb_pull_no_gain[itype]
					);
			}
			for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
					itype++) {
				seq_printf(seq, " %u %u %u %u %u %u",
				    sd->lagged_count[itype],
				    sd->lagged_little_tasks[itype],
				    sd->lagged_no_cfs_tasks[itype],
				    sd->lagged_pass_soon[itype],
				    sd->lagged_not_min[itype],
				    sd->lagged_found[itype]
					);
			}
			for (itype = CPU_IDLE; itype < CPU_MAX_IDLE_TYPES;
					itype++) {
				seq_printf(seq, " %u %u %u %u %u %u %u %u %u %u %u",
				    sd->detach_count[itype],
				    sd->detach_neg_diff[itype],
				    sd->detach_loop_stop[itype],
				    sd->detach_task_count[itype],
				    sd->detach_task_cannot[itype],
				    sd->detach_task_not_lag[itype],
				    sd->detach_task_too_lag[itype],
				    sd->detach_complete[itype],
				    sd->detach_task_detach[itype],
				    sd->detach_task_too_detach[itype],
				    sd->detach_task_not_detach[itype]
					);
			}
			seq_printf(seq, " %ld %u %ld",
					atomic64_read(&sd->vruntime->target),
					sd->target_update_racing,
					atomic64_read(&sd->vruntime->min_child) == (long) NULL
									? -1
									: atomic64_read(&sd->vruntime->min_target)
						);

			for (itype = 0; itype < NUM_MAX_TARGET_DIFF; itype++) {
				seq_printf(seq, " %u", sd->target_diff[itype]);
			}
#endif /* CONFIG_GVFS_STATS */
			seq_printf(seq, "\n");
		}
		rcu_read_unlock();
#endif
	}
	return 0;
}

/*
 * This itererator needs some explanation.
 * It returns 1 for the header position.
 * This means 2 is cpu 0.
 * In a hotplugged system some cpus, including cpu 0, may be missing so we have
 * to use cpumask_* to iterate over the cpus.
 */
static void *schedstat_start(struct seq_file *file, loff_t *offset)
{
	unsigned long n = *offset;

	if (n == 0)
		return (void *) 1;

	n--;

	if (n > 0)
		n = cpumask_next(n - 1, cpu_online_mask);
	else
		n = cpumask_first(cpu_online_mask);

	*offset = n + 1;

	if (n < nr_cpu_ids)
		return (void *)(unsigned long)(n + 2);
	return NULL;
}

static void *schedstat_next(struct seq_file *file, void *data, loff_t *offset)
{
	(*offset)++;
	return schedstat_start(file, offset);
}

static void schedstat_stop(struct seq_file *file, void *data)
{
}

static const struct seq_operations schedstat_sops = {
	.start = schedstat_start,
	.next  = schedstat_next,
	.stop  = schedstat_stop,
	.show  = show_schedstat,
};

static int schedstat_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &schedstat_sops);
}

static const struct file_operations proc_schedstat_operations = {
	.open    = schedstat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int __init proc_schedstat_init(void)
{
	proc_create("schedstat", 0, NULL, &proc_schedstat_operations);
	return 0;
}
subsys_initcall(proc_schedstat_init);

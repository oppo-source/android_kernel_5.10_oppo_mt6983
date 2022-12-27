// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/cpumask.h>
#include <linux/topology.h>
#include <linux/io.h>

#include "mtk_ppm_platform.h"
#include "mtk_ppm_internal.h"
#include "mtk_unified_power.h"

struct ppm_cobra_data *cobra_tbl;
struct ppm_cobra_lookup cobra_lookup_data;

static int Core_limit[NR_PPM_CLUSTERS] = {CORE_NUM_L, CORE_NUM_B};
int cobra_init_done;
static int is_perf_fist;

#define ACT_CORE(cluster)	(active_core[PPM_CLUSTER_##cluster])
#define CORE_LIMIT(cluster)	(core_limit_tmp[PPM_CLUSTER_##cluster])

struct ppm_cobra_data *ppm_cobra_pass_tbl(void)
{
	if (cobra_init_done)
		return cobra_tbl;
	return NULL;
}

int eara_is_perf_first(void)
{
	return is_perf_fist;
}

void eara_pass_perf_first_hint(int enable)
{
	is_perf_fist = enable;
}

static unsigned int get_idx_in_pwr_tbl(enum ppm_cluster cluster)
{
	unsigned int idx = 0;

	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("%s: Invalid input: cluster=%d\n", __func__, cluster);
		WARN_ON(1);
	}

	while (cluster)
		idx += get_cluster_max_cpu_core(--cluster);

	return idx;
}

static short get_delta_pwr(enum ppm_cluster cluster,
				unsigned int core, unsigned int opp)
{
	unsigned int idx;
	unsigned int cur_opp, prev_opp;
	int delta_pwr;

	if (core > get_cluster_max_cpu_core(cluster)
		|| opp > get_cluster_min_cpufreq_idx(cluster)) {
		ppm_err("%s: Invalid input: core=%d, opp=%d\n",
			__func__, core, opp);
		WARN_ON(1);
		return 0;
	}

	if (core == 0)
		return 0;

	idx = get_idx_in_pwr_tbl(cluster);

	cur_opp = opp;
	prev_opp = opp + 1;

	if (opp == COBRA_OPP_NUM - 1) {
		delta_pwr = (core == 1)
		? cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].power_idx
		: (cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].power_idx -
		cobra_tbl->basic_pwr_tbl[idx+core-2][cur_opp].power_idx);
	} else {
		delta_pwr =
		cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].power_idx -
		cobra_tbl->basic_pwr_tbl[idx+core-1][prev_opp].power_idx;
	}

	return delta_pwr;
}

static short get_delta_perf(enum ppm_cluster cluster, unsigned int core,
				unsigned int opp)
{
	unsigned int idx, cur_opp, prev_opp;
	short delta_perf;

	if (core > get_cluster_max_cpu_core(cluster)
		|| opp > get_cluster_min_cpufreq_idx(cluster)) {
		ppm_err("%s: Invalid input: cluster=%d, core=%d, opp=%d\n",
			__func__, cluster, core, opp);
		WARN_ON(1);
	}

	if (core == 0)
		return 0;

	idx = get_idx_in_pwr_tbl(cluster);

	cur_opp = opp;
	prev_opp = opp + 1;

	if (opp == COBRA_OPP_NUM - 1) {
		delta_perf = (core == 1)
		? cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].perf_idx
		: (cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].perf_idx -
		cobra_tbl->basic_pwr_tbl[idx+core-2][cur_opp].perf_idx);
	} else {
		delta_perf =
		cobra_tbl->basic_pwr_tbl[idx+core-1][cur_opp].perf_idx -
		cobra_tbl->basic_pwr_tbl[idx+core-1][prev_opp].perf_idx;
	}

	return delta_perf;
}

static int get_perf(enum ppm_cluster cluster, unsigned int core,
				unsigned int opp)
{
	unsigned int idx, min_idx;
	int perf;
	int ratio = 0;

	ppm_dbg(COBRA, "%s: input: cluster=%d, core=%d, opp=%d\n",
			__func__, cluster, core, opp);

	if (core > get_cluster_max_cpu_core(cluster)) {
		ppm_err("%s: Invalid input: cluster=%d, core=%d, opp=%d\n",
			__func__, cluster, core, opp);
		WARN_ON(1);
		return 0;
	}

	if (core == 0)
		return 0;

	min_idx = get_cluster_min_cpufreq_idx(cluster);

	if (opp >= min_idx) {
		opp = min_idx;
		core--;
		ratio = 100;
	}

	if (core == 0)
		core = 1;

	idx = get_idx_in_pwr_tbl(cluster);

	perf = cobra_tbl->basic_pwr_tbl[idx+core-1][opp].perf_idx *
		cobra_tbl->basic_pwr_tbl[idx][opp].perf_idx;

	if (ratio)
		perf = perf * ratio;

	ppm_dbg(COBRA, "%s: output: idx=%d, core=%d, opp=%d, perf=%d\n",
			__func__, idx, core, opp, perf);

	return perf;
}

static short get_delta_eff(enum ppm_cluster cluster, unsigned int core,
				unsigned int opp)
{
	short delta_pwr, delta_perf, delta_eff;

	if (core > get_cluster_max_cpu_core(cluster)
		|| opp > get_cluster_min_cpufreq_idx(cluster)) {
		ppm_err("%s: Invalid input: core=%d, opp=%d,  cluster=%d\n",
			__func__, core, opp, cluster);
		WARN_ON(1);
	}

	if (core == 0)
		return 0;

	delta_pwr = get_delta_pwr(cluster, core, opp);
	if (delta_pwr <= 0)
		return 0;

	delta_perf = get_delta_perf(cluster, core, opp);

	if (opp == COBRA_OPP_NUM - 1)
		/* x10 to make it hard to turn off cores */
		delta_eff = (delta_perf * 1000) / delta_pwr;
	else
		delta_eff = (delta_perf * 100) / delta_pwr;

	return delta_eff;
}


void ppm_cobra_update_core_limit(unsigned int cluster, int limit)
{
	if (cluster >= NR_PPM_CLUSTERS) {
		ppm_err("%s: Invalid cluster id = %d\n", __func__, cluster);
		WARN_ON(1);
		return;
	}

	if (limit < 0 || limit > get_cluster_max_cpu_core(cluster)) {
		ppm_err("%s: Invalid core limit for cluster%d = %d\n",
			__func__, cluster, limit);
		WARN_ON(1);
		return;
	}

	Core_limit[cluster] = limit;
}

void ppm_cobra_update_freq_limit(unsigned int cluster, int limit)
{
	/* NO NEED */
}

void ppm_cobra_update_limit(void *user_req)
{
	struct ppm_policy_req *req;
	int power_budget;
	int opp[NR_PPM_CLUSTERS];
	int active_core[NR_PPM_CLUSTERS];
#if PPM_COBRA_USE_CORE_LIMIT
	int core_limit_tmp[NR_PPM_CLUSTERS];
#endif
	int i;
	struct cpumask cluster_cpu, online_cpu;
	int delta_power;
	/* Get power index of current OPP */
	int curr_power = 0;
	struct ppm_cluster_status cl_status[NR_PPM_CLUSTERS];

	/* skip if DVFS is not ready (we cannot get current freq...) */
	/* skip if COBRA is not init yet */
	if (!ppm_main_info.client_info[PPM_CLIENT_DVFS].limit_cb ||
		!cobra_init_done)
		return;

	if (!user_req)
		return;

	req = (struct ppm_policy_req *)user_req;
	power_budget = req->power_budget;

	if (power_budget >= ppm_get_max_pwr_idx())
		return;

	ppm_dbg(COBRA, "[PREV]Core_Limit=%d%d, policy_limit=%d%d\n",
			Core_limit[PPM_CLUSTER_L],
			Core_limit[PPM_CLUSTER_B],
			req->limit[PPM_CLUSTER_L].max_cpu_core,
			req->limit[PPM_CLUSTER_B].max_cpu_core);

	for_each_ppm_clusters(i) {
		ppm_get_cl_cpus(&cluster_cpu, i);
		cpumask_and(&online_cpu, &cluster_cpu, cpu_online_mask);

		cl_status[i].core_num = cpumask_weight(&online_cpu);
		cl_status[i].volt = 0;	/* don't care */
		if (!cl_status[i].core_num)
			cl_status[i].freq_idx = -1;
		else
			cl_status[i].freq_idx = ppm_main_freq_to_idx(i,
				mt_cpufreq_get_cur_phy_freq_no_lock(i),
				CPUFREQ_RELATION_L);

		ppm_ver("[%d] core = %d, freq_idx = %d\n",
			i, cl_status[i].core_num, cl_status[i].freq_idx);
	}

#if PPM_COBRA_USE_CORE_LIMIT
	for_each_ppm_clusters(i) {
		if (cl_status[i].core_num > Core_limit[i])
			cl_status[i].core_num = Core_limit[i];
		if (req->limit[i].max_cpu_core < Core_limit[i])
			Core_limit[i] = req->limit[i].max_cpu_core;
	}
#endif

	if (cl_status[PPM_CLUSTER_L].core_num == 0 &&
		cl_status[PPM_CLUSTER_B].core_num == 0) {
		if (Core_limit[PPM_CLUSTER_L] > 0) {
			cl_status[PPM_CLUSTER_L].core_num = 1;
			cl_status[PPM_CLUSTER_L].freq_idx =
				get_cluster_max_cpufreq_idx(PPM_CLUSTER_L);
		} else {
			cl_status[PPM_CLUSTER_B].core_num = 1;
			cl_status[PPM_CLUSTER_B].freq_idx =
				get_cluster_max_cpufreq_idx(PPM_CLUSTER_B);
		}
	}


	curr_power = ppm_find_pwr_idx(cl_status);
	if (curr_power < 0)
		curr_power = mt_ppm_thermal_get_max_power();
	delta_power = power_budget - curr_power;

	for_each_ppm_clusters(i) {
		opp[i] = cl_status[i].freq_idx;

		/* Get Active Core number of each cluster */
		active_core[i] = (cl_status[i].core_num >= 0) ?
					cl_status[i].core_num : 0;

#if PPM_COBRA_USE_CORE_LIMIT
		core_limit_tmp[i] = Core_limit[i];
		req->limit[i].max_cpu_core = core_limit_tmp[i];
#endif
	}

	ppm_dbg(COBRA,
		"[IN](bgt/delta/cur)=(%d/%d/%d),(opp/act/c_lmt)=(%d,%d/%d%d/%d%d)\n",
		power_budget, delta_power, curr_power,
		opp[PPM_CLUSTER_L], opp[PPM_CLUSTER_B],
		ACT_CORE(L), ACT_CORE(B),
		CORE_LIMIT(L), CORE_LIMIT(B));


	/* increase ferquency limit */
	if (delta_power >= 0) {
		while (1) {
			int ChoosenCl = -1, MaxEff = 0, ChoosenPwr = 0;
			int target_delta_pwr, target_delta_eff;

/* give remaining power to B-cluster if L-cluster opp is 0 */
			if (opp[PPM_CLUSTER_L] == 0 && ACT_CORE(B) == 0) {
				target_delta_pwr =
					get_delta_pwr(
						PPM_CLUSTER_B,
						1,
						COBRA_OPP_NUM-1);
				if (delta_power >= target_delta_pwr) {
					ACT_CORE(B) = 1;
					delta_power -= target_delta_pwr;
					opp[PPM_CLUSTER_B] = COBRA_OPP_NUM - 1;
				}
			}

			/* B-cluster */
			if (ACT_CORE(B) > 0 && opp[PPM_CLUSTER_B] > 0) {
				target_delta_pwr =
					get_delta_pwr(
						PPM_CLUSTER_B,
						ACT_CORE(B),
						opp[PPM_CLUSTER_B]-1);
				if (delta_power >= target_delta_pwr) {
					if (is_perf_fist)
						MaxEff = get_perf(
							PPM_CLUSTER_B,
							ACT_CORE(B),
							opp[PPM_CLUSTER_B]-1);
					else
						MaxEff = get_delta_eff(
							PPM_CLUSTER_B,
							ACT_CORE(B),
							opp[PPM_CLUSTER_B]-1);
					ChoosenCl = PPM_CLUSTER_B;
					ChoosenPwr = target_delta_pwr;
				}
			}

			/* L-cluster */
			if (ACT_CORE(L) > 0 && opp[PPM_CLUSTER_L] > 0) {
				target_delta_pwr = get_delta_pwr(
							PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]-1);
				if (is_perf_fist)
					target_delta_eff = get_perf(
							PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]-1);
				else
					target_delta_eff = get_delta_eff(
							PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]-1);

				if (delta_power >= target_delta_pwr &&
						MaxEff <= target_delta_eff) {
					MaxEff = target_delta_eff;
					ChoosenCl = PPM_CLUSTER_L;
					ChoosenPwr = target_delta_pwr;
				}
			}

			if (ChoosenCl != -1)
				goto prepare_next_round;

		/* exceed power budget or all active core is highest freq. */
#if PPM_COBRA_USE_CORE_LIMIT
			/* no enough budget */
			if (opp[PPM_CLUSTER_L] != 0)
				goto end;

			if (is_perf_fist) {
				/* give budget to B */
				while (CORE_LIMIT(B) <
				get_cluster_max_cpu_core(PPM_CLUSTER_B)) {
					target_delta_pwr = get_delta_pwr(
						PPM_CLUSTER_B,
						CORE_LIMIT(B)+1,
						COBRA_OPP_NUM-1);
					if (delta_power < target_delta_pwr)
						break;

					delta_power -= target_delta_pwr;
					req->limit[PPM_CLUSTER_B].max_cpu_core =
					++CORE_LIMIT(B);
				}

				/* give budget to L */
				while (CORE_LIMIT(L) <
				get_cluster_max_cpu_core(PPM_CLUSTER_L)) {
					target_delta_pwr = get_delta_pwr(
						PPM_CLUSTER_L,
						CORE_LIMIT(L)+1,
						COBRA_OPP_NUM-1);
					if (delta_power < target_delta_pwr)
						break;

					delta_power -= target_delta_pwr;
					req->limit[PPM_CLUSTER_L].max_cpu_core =
					++CORE_LIMIT(L);
				}
			} else {
				/* give budget to L */
				while (CORE_LIMIT(L) <
				get_cluster_max_cpu_core(PPM_CLUSTER_L)) {
					target_delta_pwr = get_delta_pwr(
							PPM_CLUSTER_L,
							CORE_LIMIT(L)+1,
							COBRA_OPP_NUM-1);
					if (delta_power < target_delta_pwr)
						break;

					delta_power -= target_delta_pwr;
					req->limit[PPM_CLUSTER_L].max_cpu_core =
					++CORE_LIMIT(L);
				}

				/* give budget to B */
				while (CORE_LIMIT(B) <
				get_cluster_max_cpu_core(PPM_CLUSTER_B)) {
					target_delta_pwr = get_delta_pwr(
						PPM_CLUSTER_B,
						CORE_LIMIT(B)+1,
						COBRA_OPP_NUM-1);
					if (delta_power < target_delta_pwr)
						break;

					delta_power -= target_delta_pwr;
					req->limit[PPM_CLUSTER_B].max_cpu_core =
					++CORE_LIMIT(B);
				}
			}
end:
#endif
			ppm_dbg(COBRA,
				"[+]ChoosenCl=-1! delta=%d, (opp/c_lmt)=(%d,%d/%d%d)\n",
				delta_power,
				opp[PPM_CLUSTER_L],
				opp[PPM_CLUSTER_B],
				CORE_LIMIT(L),
				CORE_LIMIT(B));

			break;

prepare_next_round:
				opp[ChoosenCl] -= 1;
			delta_power -= ChoosenPwr;

			ppm_dbg(COBRA,
				"[+](delta/Cl/Pwr)=(%d,%d,%d), opp=%d,%d\n",
				delta_power, ChoosenCl, ChoosenPwr,
				opp[PPM_CLUSTER_L], opp[PPM_CLUSTER_B]);
		}
	} else {
		while (delta_power < 0) {
			int ChoosenCl = -1;
			int MinEff = 10000;
			/*
			 * should be bigger than max value of
			 * efficiency_* array
			 */
			int ChoosenPwr = 0;
			int target_delta_eff;

			/* B-cluster */
			if (ACT_CORE(B) > 0
			&& opp[PPM_CLUSTER_B] < PPM_COBRA_MAX_FREQ_IDX) {
				if (is_perf_fist)
					MinEff =
					get_perf(PPM_CLUSTER_B,
						ACT_CORE(B),
						opp[PPM_CLUSTER_B]);
				else
					MinEff =
					get_delta_eff(PPM_CLUSTER_B,
						ACT_CORE(B),
						opp[PPM_CLUSTER_B]);

				ChoosenCl = PPM_CLUSTER_B;
				ChoosenPwr =
					get_delta_pwr(PPM_CLUSTER_B,
						ACT_CORE(B),
						opp[PPM_CLUSTER_B]);
			}

			/* L-cluster */
			if (ACT_CORE(L) > 0
			&& opp[PPM_CLUSTER_L] < PPM_COBRA_MAX_FREQ_IDX) {
				if (is_perf_fist) {
					/*keep 1L active*/
					if (ACT_CORE(L) > 1
					|| opp[PPM_CLUSTER_L]
					!= PPM_COBRA_MAX_FREQ_IDX-1)
						target_delta_eff =
						get_perf(PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]);
					else
						target_delta_eff = 9999999;
				} else
					target_delta_eff = get_delta_eff(
							PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]);
				if (MinEff > target_delta_eff) {
					MinEff = target_delta_eff;
					ChoosenCl = PPM_CLUSTER_L;
					ChoosenPwr =
						get_delta_pwr(PPM_CLUSTER_L,
							ACT_CORE(L),
							opp[PPM_CLUSTER_L]);
				}
			}

			if (ChoosenCl == -1) {
				ppm_err("No lower OPP!(bgt/delta/cur)= ");
				ppm_err("(%d/%d/%d),(opp/act)=(%d,%d/%d%d)\n",
				power_budget, delta_power, curr_power,
				opp[PPM_CLUSTER_L], opp[PPM_CLUSTER_B],
				ACT_CORE(L), ACT_CORE(B));
				break;
			}

			/* change opp of cluster */
			opp[ChoosenCl] += 1;

			/* Turned off core */
			/* TODO: check this! */
#if PPM_COBRA_USE_CORE_LIMIT
			if (opp[PPM_CLUSTER_B] == PPM_COBRA_MAX_FREQ_IDX
			&& ACT_CORE(B) > 0) {
				req->limit[PPM_CLUSTER_B].max_cpu_core =
				--ACT_CORE(B);
				opp[PPM_CLUSTER_B] = PPM_COBRA_MAX_FREQ_IDX - 1;
			} else if (opp[PPM_CLUSTER_L] ==
				PPM_COBRA_MAX_FREQ_IDX) {
				if (ACT_CORE(L) > 1)
					req->limit[PPM_CLUSTER_L].max_cpu_core =
					--ACT_CORE(L);
				opp[PPM_CLUSTER_L] = PPM_COBRA_MAX_FREQ_IDX - 1;
			}
#endif

			delta_power += ChoosenPwr;
			curr_power -= ChoosenPwr;

			ppm_dbg(COBRA,
				"[-](delta/Cl/Pwr)=(%d,%d,%d), (opp/act)=(%d,%d/%d%d)\n",
				delta_power, ChoosenCl, ChoosenPwr,
				opp[PPM_CLUSTER_L], opp[PPM_CLUSTER_B],
				ACT_CORE(L), ACT_CORE(B));
		}
	}

	/* Set all frequency limit of the cluster */
	/* Set OPP of Cluser n to opp[n] */
	for_each_ppm_clusters(i) {
		if (i == PPM_CLUSTER_B) {
			if (opp[i] >= 0 && ACT_CORE(B) > 0)
				req->limit[i].max_cpufreq_idx = opp[i];
			else
				req->limit[i].max_cpufreq_idx =
					get_cluster_min_cpufreq_idx(i);
		} else
			req->limit[i].max_cpufreq_idx = opp[i];
	}

	ppm_dbg(COBRA,
		"[OUT]delta=%d, (opp/act/c_lmt/f_lmt)=(%d,%d/%d%d/%d%d/%d,%d)\n",
			delta_power,
			opp[PPM_CLUSTER_L], opp[PPM_CLUSTER_B],
			ACT_CORE(L), ACT_CORE(B),
			req->limit[PPM_CLUSTER_L].max_cpu_core,
			req->limit[PPM_CLUSTER_B].max_cpu_core,
			req->limit[PPM_CLUSTER_L].max_cpufreq_idx,
			req->limit[PPM_CLUSTER_B].max_cpufreq_idx);

	/* error check */
	for_each_ppm_clusters(i) {
		if (req->limit[i].max_cpufreq_idx >
		req->limit[i].min_cpufreq_idx)
			req->limit[i].min_cpufreq_idx =
			req->limit[i].max_cpufreq_idx;
		if (req->limit[i].max_cpu_core <
		req->limit[i].min_cpu_core)
			req->limit[i].min_cpu_core =
			req->limit[i].max_cpu_core;
	}
}

void ppm_cobra_init(void)
{
	int i, j;

#ifdef PPM_SSPM_SUPPORT
	/* remap sram for cobra */
	cobra_tbl = ioremap_nocache(PPM_COBRA_TBL_SRAM_ADDR,
					PPM_COBRA_TBL_SRAM_SIZE);
#else
	cobra_tbl = kzalloc(sizeof(struct ppm_cobra_data), GFP_KERNEL);
	ppm_info("cobra_tbl: %p\n", cobra_tbl);
#endif
	if (!cobra_tbl) {
		ppm_err("remap cobra_tbl failed!\n");
		WARN_ON(1);
		return;
	}

#ifdef PPM_SSPM_SUPPORT
	/* ppm_info("addr of cobra_tbl = 0x%p, size = %lu\n", */
	/* cobra_tbl, PPM_COBRA_TBL_SRAM_SIZE); */
	memset_io((u8 *)cobra_tbl, 0x00, PPM_COBRA_TBL_SRAM_SIZE);
#endif

#if IS_ENABLED(CONFIG_MTK_UNIFY_POWER)
	{
		unsigned int core, dyn, lkg, dyn_c, lkg_c, cap;

		/* generate basic power table */
		ppm_ver("basic power table:\n");
		for (i = 0; i < TOTAL_CORE_NUM; i++) {
			for (j = 0; j < DVFS_OPP_NUM; j++) {
				core = (i % CORE_NUM_L) + 1;
				dyn = upower_get_power(
					i/CORE_NUM_L, j, UPOWER_DYN);
				lkg = upower_get_power(
					i/CORE_NUM_L, j, UPOWER_LKG);
				dyn_c = upower_get_power(
					i/CORE_NUM_L + NR_PPM_CLUSTERS,
					j, UPOWER_DYN);
				lkg_c = upower_get_power(
					i/CORE_NUM_L + NR_PPM_CLUSTERS,
					j, UPOWER_LKG);
				cap = upower_get_power(
					i/CORE_NUM_L, j, UPOWER_CPU_STATES);

				cobra_tbl->basic_pwr_tbl[i][j].power_idx =
					((dyn + lkg) * core +
					(dyn_c + lkg_c)) / 1000;
				cobra_tbl->basic_pwr_tbl[i][j].perf_idx =
					cap * core;

				ppm_ver("[%d][%d] = (%d, %d)\n", i, j,
				cobra_tbl->basic_pwr_tbl[i][j].power_idx,
				cobra_tbl->basic_pwr_tbl[i][j].perf_idx);
			}
		}
	}
#else
	for (i = 0; i < TOTAL_CORE_NUM; i++) {
		for (j = 0; j < DVFS_OPP_NUM; j++) {
			cobra_tbl->basic_pwr_tbl[i][j].power_idx = 0;
			cobra_tbl->basic_pwr_tbl[i][j].perf_idx = 0;
		}
	}
#endif

	cobra_init_done = 1;

	ppm_info("COBRA init done!\n");
}


void ppm_cobra_dump_tbl(struct seq_file *m)
{
	struct ppm_cluster_status cl_status[NR_PPM_CLUSTERS];
	int i, j;
	int power;

	seq_puts(m, "\n========================================================\n");
	seq_puts(m, "(L_core, L_opp, B_core, B_opp) = power");
	seq_puts(m, "\n========================================================\n");

	/* only list power for all core online case */
	for_each_ppm_clusters(i)
		cl_status[i].core_num = get_cluster_max_cpu_core(i);

	for (i = 0; i < DVFS_OPP_NUM; i++) {
		for (j = 0; j < DVFS_OPP_NUM; j++) {
			cl_status[PPM_CLUSTER_L].freq_idx = i;
			cl_status[PPM_CLUSTER_B].freq_idx = j;

			power = ppm_find_pwr_idx(cl_status);
			if (power) {
				seq_printf(m, "(%d, %2d, %d, %2d) = %4d\n",
					cl_status[PPM_CLUSTER_L].core_num,
					cl_status[PPM_CLUSTER_L].freq_idx,
					cl_status[PPM_CLUSTER_B].core_num,
					cl_status[PPM_CLUSTER_B].freq_idx,
					power
				);
			}
		}
	}
}

static unsigned int get_limit_opp_and_budget(void)
{
	unsigned int power = 0;
	unsigned int i, j, k, idx, core;

	for (i = 0; i <= get_cluster_min_cpufreq_idx(PPM_CLUSTER_L); i++) {
		cobra_lookup_data.limit[PPM_CLUSTER_L].opp = i;
		for (j = 0;
			j <= get_cluster_min_cpufreq_idx(PPM_CLUSTER_B);
			j++) {
			cobra_lookup_data.limit[PPM_CLUSTER_B].opp = j;

			for_each_ppm_clusters(k) {
				core = cobra_lookup_data.limit[k].core;

				if (!core)
					continue;

				idx = get_idx_in_pwr_tbl(k) + core - 1;

				if (idx >= TOTAL_CORE_NUM ||
					i >= DVFS_OPP_NUM ||
					j >= DVFS_OPP_NUM) {
					ppm_info("[%p] idx: %d i:%d j:%d core:%d\n",
						cobra_tbl, idx, i, j, core);
					return 0;
				}

				power += (k == PPM_CLUSTER_B)
				? cobra_tbl->basic_pwr_tbl[idx][j].power_idx
				: cobra_tbl->basic_pwr_tbl[idx][i].power_idx;
			}

			if (power <= cobra_lookup_data.budget)
				return power;

			power = 0;
		}
	}

	return power;
}

static void ppm_cobra_lookup_by_budget(struct seq_file *m)
{
	int i, j;
	unsigned int power;

	seq_puts(m, "\n========================================================\n");
	seq_puts(m, "(L_core, L_opp, B_core, B_opp) = power");
	seq_puts(m, "\n========================================================\n");

	seq_printf(m, "Input budget = %d\n\n", cobra_lookup_data.budget);

	for (i = get_cluster_max_cpu_core(PPM_CLUSTER_B); i >= 0; i--) {
		for (j = get_cluster_max_cpu_core(PPM_CLUSTER_L); j >= 0; j--) {
			if (!i && !j)
				continue;

			cobra_lookup_data.limit[PPM_CLUSTER_L].core = j;
			cobra_lookup_data.limit[PPM_CLUSTER_B].core = i;
			power = get_limit_opp_and_budget();

			if (power) {
				seq_printf(m, "(%d, %2d, %d, %2d) = %4d\n",
				j,
				cobra_lookup_data.limit[PPM_CLUSTER_L].opp,
				i,
				cobra_lookup_data.limit[PPM_CLUSTER_B].opp,
				power);
			}
		}
	}
}

static void ppm_cobra_lookup_by_limit(struct seq_file *m)
{
	unsigned int budget = 0, core, opp, i;

	for_each_ppm_clusters(i) {
		core = (cobra_lookup_data.limit[i].core >
			get_cluster_max_cpu_core(i)) ?
			get_cluster_max_cpu_core(i) :
			cobra_lookup_data.limit[i].core;
		opp = (cobra_lookup_data.limit[i].opp >
			get_cluster_min_cpufreq_idx(i)) ?
			get_cluster_min_cpufreq_idx(i) :
			cobra_lookup_data.limit[i].opp;

		if (core)
			budget += cobra_tbl->basic_pwr_tbl
				[CORE_NUM_L*i+core-1][opp].power_idx;

		seq_printf(m,
			"Cluster %d: core = %d, opp = %d\n", i, core, opp);
	}

	seq_printf(m, "Budget = %d mW\n", budget);
}

extern void ppm_cobra_lookup_get_result(
		struct seq_file *m, enum ppm_cobra_lookup_type type)
{
	switch (type) {
	case LOOKUP_BY_BUDGET:
		ppm_cobra_lookup_by_budget(m);
		break;
	case LOOKUP_BY_LIMIT:
		ppm_cobra_lookup_by_limit(m);
		break;
	default:
		ppm_err("Invalid lookup type %d\n", type);
		break;
	}
}

// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/slab.h>

#include <linux/debugfs.h>

#include "ged_base.h"
#include "ged_debugFS.h"

#define GED_DEBUGFS_DIR_NAME "ged"

static struct dentry *gpsDebugFSEntryDir;

struct GED_DEBUGFS_PRIV_DATA {
	const struct seq_operations *psReadOps;
	ssize_t (*pfnWrite)(const char __user *pszBuffer, size_t uiCount,
		loff_t uiPosition, void *pvData);
	void *pvData;
};
//-----------------------------------------------------------------------------
static int ged_debugFS_open(struct inode *psINode, struct file *psFile)
{
	struct GED_DEBUGFS_PRIV_DATA *psPrivData =
		(struct GED_DEBUGFS_PRIV_DATA *)psINode->i_private;
	int iResult;

	iResult = seq_open(psFile, psPrivData->psReadOps);
	if (iResult == 0) {
		struct seq_file *psSeqFile = psFile->private_data;

		psSeqFile->private = psPrivData->pvData;
		return GED_OK;
	}

	return GED_ERROR_FAIL;
}
//-----------------------------------------------------------------------------
static ssize_t ged_debugFS_write(
		struct file        *psFile,
		const char __user  *pszBuffer,
		size_t              uiCount,
		loff_t             *puiPosition)
{
	struct inode *psINode = psFile->f_path.dentry->d_inode;
	struct GED_DEBUGFS_PRIV_DATA *psPrivData =
		(struct GED_DEBUGFS_PRIV_DATA *)psINode->i_private;

	if (psPrivData->pfnWrite == NULL)
		return -EIO;

	return psPrivData->pfnWrite(pszBuffer, uiCount,
		*puiPosition, psPrivData->pvData);
}
//-----------------------------------------------------------------------------
static const struct file_operations gsGEDDebugFSFileOps = {
	.owner = THIS_MODULE,
	.open = ged_debugFS_open,
	.read = seq_read,
	.write = ged_debugFS_write,
	.llseek = seq_lseek,
	.release = seq_release,
};
//-----------------------------------------------------------------------------
GED_ERROR ged_debugFS_create_entry(
		const char             *pszName,
		void                   *pvDir,
		const struct seq_operations *psReadOps,
		ssize_t (*pfnWrite)(const char __user *pszBuffer,
			size_t uiCount, loff_t uiPosition, void *pvData),
		void                   *pvData,
		struct dentry         **ppsEntry)
{
	struct GED_DEBUGFS_PRIV_DATA *psPrivData;
	struct dentry *psEntry;
	umode_t uiMode;


	psPrivData = ged_alloc(sizeof(struct GED_DEBUGFS_PRIV_DATA));
	if (psPrivData == NULL)
		return GED_ERROR_OOM;

	psPrivData->psReadOps = psReadOps;
	psPrivData->pfnWrite = pfnWrite;
	psPrivData->pvData = pvData;

	uiMode = S_IFREG;

	if (psReadOps != NULL)
		uiMode |= 0440;

	if (pfnWrite != NULL)
		uiMode |= 0220;

	psEntry = debugfs_create_file(pszName,
		uiMode,
		(pvDir != NULL) ? (struct dentry *)pvDir : gpsDebugFSEntryDir,
		psPrivData,
		&gsGEDDebugFSFileOps);
	if (IS_ERR(psEntry)) {
		GED_LOGE("Failed to create '%s' debugfs entry\n", pszName);
		return GED_ERROR_FAIL;
	}

	*ppsEntry = psEntry;

	return GED_OK;
}
//-----------------------------------------------------------------------------
void ged_debugFS_remove_entry(struct dentry *psEntry)
{
	if (psEntry != NULL && psEntry->d_inode->i_private != NULL)
		ged_free(psEntry->d_inode->i_private,
		sizeof(struct GED_DEBUGFS_PRIV_DATA));


	debugfs_remove(psEntry);
}
//-----------------------------------------------------------------------------
GED_ERROR ged_debugFS_create_entry_dir(
		const char     *pszName,
		struct dentry  *psParentDir,
		struct dentry **ppsDir)
{
	struct dentry *psDir;

	if (pszName == NULL || ppsDir == NULL)
		return GED_ERROR_INVALID_PARAMS;


	psDir = debugfs_create_dir(pszName,
		(psParentDir) ? psParentDir : gpsDebugFSEntryDir);
	if (psDir == NULL) {
		GED_LOGE("Failed to create '%s' debugfs directory\n", pszName);
		return GED_ERROR_OOM;
	}

	*ppsDir = psDir;

	return GED_OK;
}
//-----------------------------------------------------------------------------
void ged_debugFS_remove_entry_dir(struct dentry *psDir)
{
	debugfs_remove(psDir);
}
//-----------------------------------------------------------------------------
GED_ERROR ged_debugFS_init(void)
{
	//assert(gpkDebugFSEntryDir == NULL);

	gpsDebugFSEntryDir = debugfs_create_dir(GED_DEBUGFS_DIR_NAME, NULL);
	if (gpsDebugFSEntryDir == NULL) {
		GED_LOGE("Failed to create '%s' debugfs root directory\n",
			GED_DEBUGFS_DIR_NAME);
		return GED_ERROR_OOM;
	}

	return GED_OK;
}
//-----------------------------------------------------------------------------
void ged_debugFS_exit(void)
{
	//assert(gpkDebugFSEntryDir != NULL);

	debugfs_remove(gpsDebugFSEntryDir);
	gpsDebugFSEntryDir = NULL;
}
//-----------------------------------------------------------------------------


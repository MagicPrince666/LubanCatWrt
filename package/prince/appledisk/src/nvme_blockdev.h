/*
 * Copyright (C) 2012 Apple Inc. All rights reserved.
 *
 * This document is the property of Apple Inc.
 * It is considered confidential and proprietary.
 *
 * This document may not be reproduced or transmitted in any form,
 * in whole or in part, without the express written permission of
 * Apple Inc.
 */
#ifndef NVME_BLOCKDEV_H
#define NVME_BLOCKDEV_H

int nvme_blockdev_init_boot(int nvme_id);
int nvme_blockdev_init_normal(int nvme_id);

#endif // NVME_BLOCKDEV_H

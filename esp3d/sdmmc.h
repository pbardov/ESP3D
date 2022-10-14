#ifndef SDMMC_H_
#define SDMMC_H_

#include <SD_MMC.h>

namespace fs {

class SDMMCFS_EX : public SDMMCFS {
    public:
        using SDMMCFS::_card;
};

}

extern fs::SDMMCFS_EX & SD_MMC_EX;

#endif

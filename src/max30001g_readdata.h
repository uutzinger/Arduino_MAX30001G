#ifndef MAX30001G_FIFO_H
#define MAX30001G_FIFO_H

class MAX30001G {
    private:
        void  FIFOReset(void); 
        void  readECG_FIFO(bool reportRaw);
        float readECG(bool reportRaw);
        void  readBIOZ_FIFO(bool reportRaw);
        float readBIOZ(bool reportRaw);
        void  readHRandRR(void);
};

#endif

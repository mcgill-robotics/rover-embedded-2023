class RingBuffer512
{
    short buf[512];   /* array to be treated as circular buffer of BUFLEN integers */
    short write_idx = 0;       /* write index */
    public:
        void put(short data)
        {
            buf[write_idx++] = data;
            write_idx %= 512;
        }
        short *getBuffer()
        {
            return buf;
        };
};

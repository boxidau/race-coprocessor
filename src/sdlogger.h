    
class SDLogger {
    public:
        static bool ensureInitialized();
        static bool isInitialized();
        static bool createLogfile(char fileSuffix, FsFile& file);
        static bool preAlloc(FsFile& file, size_t size);
};

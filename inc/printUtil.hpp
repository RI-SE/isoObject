
#define PRINT_STRUCT(type, pInstance, ...) \
    {                                \
        std::cout << #type << " "    \
                  << #pInstance      \
                  << std::endl;      \
        type *pStr = pInstance;      \
        __VA_ARGS__                  \
        std::cout << std::endl;      \
    }

#define PRINT_FIELD(type, name)           \
    {                               \
        std::cout << "    "         \
                  << #type << " "   \
                  << #name << " = " \
                  << pStr->name     \
                  << std::endl;     \
    }

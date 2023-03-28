#ifndef COMMON_INTERFACE_LOGGING_H
#define COMMON_INTERFACE_LOGGING_H

#include <cstdint>
#include <string>
#include <atomic>
#include <functional>
#include <vector>
#include <ostream>

// 新的logging抽象层
// https://www.drdobbs.com/cpp/logging-in-c/201804215
namespace arcs {
enum LogLevel
{
    /**
     * An error occurred from which the library cannot
     * recover.  This usually indicates a programming error
     * in the code which calls the library, especially when
     * compiled in debug mode.
     */
    LOGLEVEL_FATAL,

    /**
     * An error occurred which should never happen during normal use.
     */
    LOGLEVEL_ERROR,

    /**
     * Warns about issues that, although not technically a
     * problem now, could cause problems in the future.  For
     * example, a warning will be printed when parsing a
     * message that is near the message size limit.
     */
    LOGLEVEL_WARNING,

    /**
     * Informational
     */
    LOGLEVEL_INFO,

    LOGLEVEL_DEBUG, // Debug ouput

    LOGLEVEL_BACKTRACE, // Debug ouput
};

using LogHandler =
    std::function<void(int /*level*/, const char * /*filename*/, int /*line*/,
                       const std::string & /*message*/)>;

constexpr const char *str_end(const char *str)
{
    return *str ? str_end(str + 1) : str;
}

constexpr bool str_slant(const char *str)
{
    return *str == '/' ? true : (*str ? str_slant(str + 1) : false);
}

constexpr const char *r_slant(const char *str)
{
    return *str == '/' ? (str + 1) : r_slant(str - 1);
}
constexpr const char *file_name(const char *str)
{
    return str_slant(str) ? r_slant(str_end(str)) : str;
}

template <class Dummy>
struct BaseClass_statics
{
    static LogHandler log_handler;
};

template <class Dummy>
LogHandler BaseClass_statics<Dummy>::log_handler =
    [](int level, const char *filename, int line, const std::string &message) {
        static const char *level_names[] = { "Critical", "Error", "Warning",
                                             "Info",     "Debug", "BackTrace" };

        // We use fprintf() instead of cerr because we want this to work
        // at static initialization time.
        fprintf(stderr, "[%s] [%s:%d] %s\n", level_names[level], filename, line,
                message.c_str());
        fflush(stderr); // Needed on MSVC.
    };

class BaseClass : public BaseClass_statics<void>
{
};

inline LogHandler SetDefaultLogHandler(const LogHandler &new_func)
{
    auto old_handle = BaseClass::log_handler;
    BaseClass::log_handler = new_func;
    return old_handle;
}

inline LogHandler GetDefaultLogHandler()
{
    return BaseClass::log_handler;
}

class Log;
class LogBuf : public std::basic_streambuf<char, std::char_traits<char>>
{
public:
    LogBuf(const LogHandler &handler)
    {
        if (handler) {
            handler_ = handler;
        } else {
            // 默认
            handler_ = GetDefaultLogHandler();
        }
    }

protected:
    int sync() override
    {
        if (buffer_.length()) {
            handler_(priority_, filename_, line_, buffer_);
            buffer_.erase();
        }
        return 0;
    }

    int overflow(int c) override
    {
        if (c != EOF) {
            buffer_ += static_cast<char>(c);
        } else {
            sync();
        }
        return c;
    }

private:
    friend class Log;
    std::string buffer_;
    int priority_;
    LogHandler handler_;
    const char *filename_;
    int line_;
};

class Log
{
public:
    Log(const LogHandler &handler) : os(new LogBuf(handler)) {}

    virtual ~Log()
    {
        os.flush();
        delete os.rdbuf();
    }

    std::ostream &Get(LogLevel level, const char *filename, int line)
    {
        static_cast<LogBuf *>(os.rdbuf())->priority_ = (int)level;
        static_cast<LogBuf *>(os.rdbuf())->filename_ = filename;
        static_cast<LogBuf *>(os.rdbuf())->line_ = line;
        return os;
    }

private:
    std::ostream os;
};

// 使用之前需要先定义宏  LOG_HANDLER
#define LOG_HANDLER log_handler_
#define LOGGING(level)     \
    arcs::Log(LOG_HANDLER) \
        .Get(arcs::LOGLEVEL_##level, arcs::file_name(__FILE__), __LINE__)

} // namespace arcs

#endif // COMMON_INTERFACE_LOGGING_H

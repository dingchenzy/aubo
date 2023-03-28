#ifndef AUBO_SDK_SYSTEM_ERROR_H
#define AUBO_SDK_SYSTEM_ERROR_H

#define SYSTEM_ERRORS                                                     \
    _D(DEBUG, codeCompose(0, 0, 0), "Debug message " _PH1_, "suggest...") \
    _D(POPUP, codeCompose(0, 0, 1), _PH1_, "suggest...")                  \
    _D(POPUP_DISMISS, codeCompose(0, 0, 2), _PH1_, "suggest...")

#endif // AUBO_SDK_SYSTEM_ERROR_H

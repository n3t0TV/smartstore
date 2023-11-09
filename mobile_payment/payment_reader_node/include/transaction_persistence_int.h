#pragma once

#include <mutex>
#include <string>


class TransactionPersistenceInt
{
    public:
        TransactionPersistenceInt(int min_val, int max_val, int default_val,
                                  std::string key);

        void UpdateInt(int val);

        int GetInt(void);

    private:
        const int kMinVal;

        const int kMaxVal;

        const int kDefaultVal;

        std::string kPersistenceIntKey;

        bool int_read_ = false;

        int current_val_;

        std::recursive_mutex int_mutex_;

        int GetValidInt(int val);
};

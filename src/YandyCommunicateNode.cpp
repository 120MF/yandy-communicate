#include <OF/lib/Node/Node.hpp>
#include <YandyCommunicateData.hpp>

using namespace OF;

// Register Topic for data communication
ONE_TOPIC_REGISTER(YandyCommunicateData, topic_yandy_communicate, "yandy_communicate_data");

class YandyCommunicateNode : public Node<YandyCommunicateNode> {
public:
    struct Meta {
        static constexpr size_t stack_size = 2048;
        static constexpr int priority = 5;
        static constexpr const char *name = "yandy_communicate";
    };

    struct Config {};
    inline static Config config = {};

    bool init() {
        // TODO: Initialize your node here
        return true;
    }

    void run() {
        // TODO: Implement your node's main loop
        int counter = 0;
        while (true) {
            // Example: publish data to topic
            topic_yandy_communicate.write({ counter });

            k_msleep(100);
            counter++;
        }
    }

    void cleanup() {
        // TODO: Clean up resources
    }
};

// Register the Node with OneFramework
ONE_NODE_REGISTER(YandyCommunicateNode);

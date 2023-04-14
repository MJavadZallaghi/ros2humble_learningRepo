#include "rlccpp/rlccpp.hpp"

int main(int argc, char* argv){
    rlccpp::init(argc, argv);

    auto node = rlccpp::Node::make_shared("simple_node");

    rlccpp::spin(node);

    rlccpp::shutdown(node);
}
#include <stdexcept>

#include <gtest/gtest.h>

#include "gesture_expressions.hpp"

using hexa::display::parseGestureExpressions;

TEST(GestureExpressions, OnlyGesturesWithAnExpressionAreMapped) {
    const auto map = parseGestureExpressions(R"(
gestures:
  - id: wave
    expression: Happy
  - id: bow
)");
    ASSERT_EQ(map.size(), 1u);
    EXPECT_EQ(map.at("wave"), Expression::HAPPY);
}

TEST(GestureExpressions, UnknownNameThrows) {
    EXPECT_THROW(parseGestureExpressions("gestures:\n  - id: wave\n    expression: grin\n"),
                 std::runtime_error);
}

TEST(GestureExpressions, NoGesturesIsEmpty) {
    EXPECT_TRUE(parseGestureExpressions("stand_step: {height: 0.01, time: 0.15}\n").empty());
}

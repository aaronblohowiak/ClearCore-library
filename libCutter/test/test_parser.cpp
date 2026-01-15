/**
 * @file test_parser.cpp
 * @brief Command parser unit tests
 */

#include <gtest/gtest.h>
#include "CommandParser.h"

using namespace Cutter;

// === Empty/Comment Tests ===

TEST(Parser, EmptyLine) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_FALSE(parser.Parse("", &cmd));
}

TEST(Parser, WhitespaceOnly) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_FALSE(parser.Parse("   ", &cmd));
    EXPECT_FALSE(parser.Parse("\t\t", &cmd));
    EXPECT_FALSE(parser.Parse("  \t  ", &cmd));
}

TEST(Parser, NewlineOnly) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_FALSE(parser.Parse("\n", &cmd));
    EXPECT_FALSE(parser.Parse("\r\n", &cmd));
}

TEST(Parser, Comment) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_FALSE(parser.Parse("# this is a comment", &cmd));
    EXPECT_FALSE(parser.Parse("  # indented comment", &cmd));
}

TEST(Parser, CommentAfterCommand) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping # trailing comment", &cmd));
    EXPECT_STREQ(cmd.name, "ping");
    EXPECT_EQ(cmd.param_count, 0u);
}

// === Simple Command Tests ===

TEST(Parser, SimpleCommand) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping", &cmd));
    EXPECT_STREQ(cmd.name, "ping");
    EXPECT_EQ(cmd.param_count, 0u);
}

TEST(Parser, SimpleCommandWithNewline) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping\n", &cmd));
    EXPECT_STREQ(cmd.name, "ping");
}

TEST(Parser, CommandWithLeadingWhitespace) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("   ping", &cmd));
    EXPECT_STREQ(cmd.name, "ping");
}

// === Command with Parameters ===

TEST(Parser, CommandWithOneParam) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("enable motor=0", &cmd));
    EXPECT_STREQ(cmd.name, "enable");
    EXPECT_EQ(cmd.param_count, 1u);

    int32_t motor;
    EXPECT_TRUE(cmd.GetInt("motor", &motor));
    EXPECT_EQ(motor, 0);
}

TEST(Parser, CommandWithMultipleParams) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("move motor=0 steps=1000", &cmd));
    EXPECT_STREQ(cmd.name, "move");
    EXPECT_EQ(cmd.param_count, 2u);

    int32_t motor, steps;
    EXPECT_TRUE(cmd.GetInt("motor", &motor));
    EXPECT_TRUE(cmd.GetInt("steps", &steps));
    EXPECT_EQ(motor, 0);
    EXPECT_EQ(steps, 1000);
}

TEST(Parser, NegativeValues) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("move motor=0 steps=-500", &cmd));

    int32_t steps;
    EXPECT_TRUE(cmd.GetInt("steps", &steps));
    EXPECT_EQ(steps, -500);
}

TEST(Parser, LargeValues) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("move motor=0 steps=2000000000", &cmd));

    int32_t steps;
    EXPECT_TRUE(cmd.GetInt("steps", &steps));
    EXPECT_EQ(steps, 2000000000);
}

// === Epoch and Sequence ===

TEST(Parser, EpochAndSeq) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("move epoch=2 seq=42 motor=0 steps=100", &cmd));

    EXPECT_TRUE(cmd.has_epoch);
    EXPECT_TRUE(cmd.has_seq);
    EXPECT_EQ(cmd.epoch, 2u);
    EXPECT_EQ(cmd.seq, 42u);

    // epoch/seq should NOT be in regular params
    EXPECT_EQ(cmd.param_count, 2u);
    EXPECT_FALSE(cmd.HasParam("epoch"));
    EXPECT_FALSE(cmd.HasParam("seq"));
}

TEST(Parser, SeqOnly) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping seq=1", &cmd));

    EXPECT_FALSE(cmd.has_epoch);
    EXPECT_TRUE(cmd.has_seq);
    EXPECT_EQ(cmd.seq, 1u);
}

TEST(Parser, EpochOnly) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping epoch=5", &cmd));

    EXPECT_TRUE(cmd.has_epoch);
    EXPECT_FALSE(cmd.has_seq);
    EXPECT_EQ(cmd.epoch, 5u);
}

// === Parameter Type Extraction ===

TEST(Parser, GetIntMissing) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("ping", &cmd));

    int32_t value;
    EXPECT_FALSE(cmd.GetInt("missing", &value));
}

TEST(Parser, GetIntInvalid) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test value=abc", &cmd));

    int32_t value;
    EXPECT_FALSE(cmd.GetInt("value", &value));
}

TEST(Parser, GetUInt) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test value=12345", &cmd));

    uint32_t value;
    EXPECT_TRUE(cmd.GetUInt("value", &value));
    EXPECT_EQ(value, 12345u);
}

TEST(Parser, GetUIntNegative) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test value=-1", &cmd));

    uint32_t value;
    EXPECT_FALSE(cmd.GetUInt("value", &value));  // Negative should fail
}

TEST(Parser, GetBoolTrue) {
    CommandParser parser;
    ParsedCommand cmd;

    EXPECT_TRUE(parser.Parse("test a=1 b=true c=TRUE", &cmd));

    bool a, b, c;
    EXPECT_TRUE(cmd.GetBool("a", &a));
    EXPECT_TRUE(cmd.GetBool("b", &b));
    EXPECT_TRUE(cmd.GetBool("c", &c));
    EXPECT_TRUE(a);
    EXPECT_TRUE(b);
    EXPECT_TRUE(c);
}

TEST(Parser, GetBoolFalse) {
    CommandParser parser;
    ParsedCommand cmd;

    EXPECT_TRUE(parser.Parse("test a=0 b=false c=FALSE", &cmd));

    bool a, b, c;
    EXPECT_TRUE(cmd.GetBool("a", &a));
    EXPECT_TRUE(cmd.GetBool("b", &b));
    EXPECT_TRUE(cmd.GetBool("c", &c));
    EXPECT_FALSE(a);
    EXPECT_FALSE(b);
    EXPECT_FALSE(c);
}

TEST(Parser, GetBoolInvalid) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test value=maybe", &cmd));

    bool value;
    EXPECT_FALSE(cmd.GetBool("value", &value));
}

TEST(Parser, GetString) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test name=hello", &cmd));

    const char* value = cmd.GetString("name");
    EXPECT_NE(value, nullptr);
    EXPECT_STREQ(value, "hello");
}

TEST(Parser, GetStringMissing) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test name=hello", &cmd));

    EXPECT_EQ(cmd.GetString("missing"), nullptr);
}

TEST(Parser, HasParam) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test a=1 b=2", &cmd));

    EXPECT_TRUE(cmd.HasParam("a"));
    EXPECT_TRUE(cmd.HasParam("b"));
    EXPECT_FALSE(cmd.HasParam("c"));
}

TEST(Parser, GetIntOr) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test a=42", &cmd));

    EXPECT_EQ(cmd.GetIntOr("a", 0), 42);
    EXPECT_EQ(cmd.GetIntOr("missing", 99), 99);
}

TEST(Parser, GetBoolOr) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test a=1", &cmd));

    EXPECT_EQ(cmd.GetBoolOr("a", false), true);
    EXPECT_EQ(cmd.GetBoolOr("missing", true), true);
    EXPECT_EQ(cmd.GetBoolOr("missing", false), false);
}

// === Quoted Values ===

TEST(Parser, QuotedValue) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("error message=\"error message\"", &cmd));

    const char* msg = cmd.GetString("message");
    EXPECT_NE(msg, nullptr);
    EXPECT_STREQ(msg, "error message");
}

// === Edge Cases ===

TEST(Parser, EmptyValue) {
    CommandParser parser;
    ParsedCommand cmd;
    EXPECT_TRUE(parser.Parse("test value=", &cmd));

    const char* value = cmd.GetString("value");
    EXPECT_NE(value, nullptr);
    EXPECT_STREQ(value, "");
}

TEST(Parser, ManyParams) {
    CommandParser parser;
    ParsedCommand cmd;

    // Build command with many params
    EXPECT_TRUE(parser.Parse(
        "test a=1 b=2 c=3 d=4 e=5 f=6 g=7 h=8 i=9 j=10 k=11 l=12", &cmd));

    EXPECT_EQ(cmd.param_count, 12u);
}

TEST(Parser, TooManyParams) {
    CommandParser parser;
    ParsedCommand cmd;

    // MAX_PARAMS is 16, so 20 params should overflow
    EXPECT_TRUE(parser.Parse(
        "test a=1 b=2 c=3 d=4 e=5 f=6 g=7 h=8 i=9 j=10 "
        "k=11 l=12 m=13 n=14 o=15 p=16 q=17 r=18 s=19 t=20", &cmd));

    // Should only have MAX_PARAMS stored
    EXPECT_EQ(cmd.param_count, MAX_PARAMS);
}

TEST(Parser, InvalidKeyValueFirst) {
    CommandParser parser;
    ParsedCommand cmd;

    // First token is key=value, should be rejected
    EXPECT_FALSE(parser.Parse("key=value", &cmd));
}

TEST(Parser, LongCommandName) {
    CommandParser parser;
    ParsedCommand cmd;

    EXPECT_TRUE(parser.Parse("configure_digital_in pin=6 report_changes=1", &cmd));
    EXPECT_STREQ(cmd.name, "configure_digital_in");
}

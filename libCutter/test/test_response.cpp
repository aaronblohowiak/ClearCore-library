/**
 * @file test_response.cpp
 * @brief Response writer unit tests
 */

#include <gtest/gtest.h>
#include "CutterResponse.h"

using namespace Cutter;

// === Ok Response Tests ===

TEST(Response, Ok) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok();
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok\n");
}

TEST(Response, OkWithIntParam) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("pin", 6);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok pin=6\n");
}

TEST(Response, OkWithMultipleParams) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("pin", 6).Param("value", 1);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok pin=6 value=1\n");
}

TEST(Response, OkWithNegativeInt) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("steps", static_cast<int32_t>(-500));
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok steps=-500\n");
}

TEST(Response, OkWithUnsignedInt) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("seq", static_cast<uint32_t>(42));
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok seq=42\n");
}

TEST(Response, OkWithBoolTrue) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("enabled", true);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok enabled=1\n");
}

TEST(Response, OkWithBoolFalse) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("enabled", false);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok enabled=0\n");
}

TEST(Response, OkWithStringParam) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("state", "ready");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok state=ready\n");
}

TEST(Response, OkWithCommandId) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{0, 5};
    w.Ok(id);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok epoch=0 seq=5\n");
}

TEST(Response, OkWithCommandIdAndParams) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{2, 7};
    w.Ok(id).Param("motor", 0);
    const char* result = w.Finish();
    // epoch/seq lead, directly after the verb
    EXPECT_STREQ(result, "ok epoch=2 seq=7 motor=0\n");
}

// === Error Response Tests ===

TEST(Response, Error) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Error(100, "Unknown command");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "error code=100 message=\"Unknown command\"\n");
}

TEST(Response, ErrorWithParams) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Error(101, "Invalid parameter").Param("param", "motor");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "error code=101 message=\"Invalid parameter\" param=motor\n");
}

TEST(Response, ErrorWithCommandId) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{1, 9};
    w.Error(100, "Unknown command", id);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "error code=100 message=\"Unknown command\" epoch=1 seq=9\n");
}

// === Event Response Tests ===

TEST(Response, Event) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Event("done");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "event type=done\n");
}

TEST(Response, EventWithParams) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Event("done").Param("motor", 0).Param("seq", static_cast<uint32_t>(42));
    const char* result = w.Finish();
    EXPECT_STREQ(result, "event type=done motor=0 seq=42\n");
}

TEST(Response, EventInput) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Event("input").Param("pin", 6).Param("value", 1);
    const char* result = w.Finish();
    EXPECT_STREQ(result, "event type=input pin=6 value=1\n");
}

TEST(Response, EventThreshold) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Event("threshold").Param("pin", 9).Param("value", static_cast<int32_t>(950));
    const char* result = w.Finish();
    EXPECT_STREQ(result, "event type=threshold pin=9 value=950\n");
}

TEST(Response, EventError) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Event("error").Param("code", static_cast<uint32_t>(200)).Param("message", "Motor fault");
    const char* result = w.Finish();
    // Note: "Motor fault" contains a space so it would be quoted
    EXPECT_STREQ(result, "event type=error code=200 message=\"Motor fault\"\n");
}

TEST(Response, EventWithCommandId) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{0, 5};
    w.Event("done", id).Param("motor", 0).Param("position", static_cast<int32_t>(1000));
    const char* result = w.Finish();
    // epoch/seq lead, directly after the type
    EXPECT_STREQ(result, "event type=done epoch=0 seq=5 motor=0 position=1000\n");
}

// === Status Response Tests ===

TEST(Response, Status) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Status("pin");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "status type=pin\n");
}

TEST(Response, StatusWithCommandId) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{1, 7};
    w.Status("motor", id)
        .Param("motor", 0)
        .Param("motor_type", "clearpath")
        .Param("enabled", true);
    const char* result = w.Finish();
    // epoch/seq lead, directly after the type; motor_type avoids colliding with type
    EXPECT_STREQ(result, "status type=motor epoch=1 seq=7 motor=0 motor_type=clearpath enabled=1\n");
}

TEST(Response, StatusSummary) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    CommandId id{1, 7};
    w.Status("summary", id)
        .Param("pin_count", static_cast<int32_t>(0))
        .Param("motor_count", static_cast<int32_t>(0));
    const char* result = w.Finish();
    EXPECT_STREQ(result, "status type=summary epoch=1 seq=7 pin_count=0 motor_count=0\n");
}

// === Debug Response Tests ===

TEST(Response, Debug) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Debug("entering state READY");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "debug message=\"entering state READY\"\n");
}

// === Buffer Overflow Tests ===

TEST(Response, SmallBuffer) {
    char buffer[10];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("very_long_param_name", 12345);
    w.Finish();

    EXPECT_TRUE(w.Overflowed());
    // Buffer should still be null-terminated
    EXPECT_EQ(buffer[sizeof(buffer) - 1], '\0');
}

TEST(Response, ExactFit) {
    // "ok\n" is 3 chars + null = 4 bytes
    char buffer[4];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok();
    const char* result = w.Finish();

    EXPECT_FALSE(w.Overflowed());
    EXPECT_STREQ(result, "ok\n");
}

TEST(Response, OneByteShort) {
    // "ok\n" needs 4 bytes (including null)
    // With only 3 bytes, can't fit "ok\n" + null
    char buffer[3];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok();
    w.Finish();

    EXPECT_TRUE(w.Overflowed());
}

// === Reset Tests ===

TEST(Response, Reset) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));

    w.Ok().Param("first", 1);
    w.Finish();
    EXPECT_STREQ(buffer, "ok first=1\n");

    w.Reset();
    w.Error(100, "test");
    w.Finish();
    EXPECT_STREQ(buffer, "error code=100 message=\"test\"\n");
}

TEST(Response, MultipleFinish) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok();

    const char* r1 = w.Finish();
    const char* r2 = w.Finish();

    // Multiple Finish calls should return same result, not add more newlines
    EXPECT_STREQ(r1, "ok\n");
    EXPECT_STREQ(r2, "ok\n");
}

// === Length Tests ===

TEST(Response, Length) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok();

    // Before Finish, length should be 2 ("ok")
    EXPECT_EQ(w.Length(), 2u);

    w.Finish();

    // After Finish, length should be 3 ("ok\n")
    EXPECT_EQ(w.Length(), 3u);
}

// === String Quoting Tests ===

TEST(Response, StringWithSpace) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("message", "hello world");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok message=\"hello world\"\n");
}

TEST(Response, StringNoSpace) {
    char buffer[256];
    ResponseWriter w(buffer, sizeof(buffer));
    w.Ok().Param("state", "ready");
    const char* result = w.Finish();
    EXPECT_STREQ(result, "ok state=ready\n");
}

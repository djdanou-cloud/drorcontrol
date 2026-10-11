// Runtime check of the generated singleton (needs Qt 6):
//   QT_QPA_PLATFORM=offscreen qmltestrunner -import ui/tokens/generated/qml -input ui/tokens/tests
import QtQuick
import QtTest
import DrorControl.Tokens

TestCase {
    name: "Tokens"

    function cleanup() { Tokens.theme = "standard" }

    function test_standard_defaults() {
        compare(Tokens.theme, "standard")
        compare(Tokens.canvas.toString(), "#212121")
        compare(Tokens.textTelemetry.toString(), "#fafafa")
        compare(Tokens.stateCaution.toString(), "#ff9400")
        fuzzyCompare(Tokens.surfaceGlass.a, 0.9, 0.003)
        compare(Tokens.sizeTelemetryL, 28)
        compare(Tokens.targetMin, 48)
        compare(Tokens.fontTelemetry, "JetBrains Mono")
    }

    function test_runtime_theme_switch() {
        Tokens.theme = "daylight"
        compare(Tokens.canvas.toString(), "#fafafa")
        compare(Tokens.stateCaution.toString(), "#a85900")
        compare(Tokens.sizeTelemetryL, 30)          // Daylight: +2 pt
        compare(Tokens.strokeBold, 2.5 * 1.25)      // Daylight: +25 % strokes
        compare(Tokens.targetMin, 56)
        Tokens.theme = "nightOps"
        compare(Tokens.canvas.toString(), "#000000")
        compare(Tokens.text.toString(), "#ff4d4d")
        Tokens.theme = "highContrast"
        compare(Tokens.text.toString(), "#ffffff")
    }

    function test_unknown_theme_falls_back_to_standard() {
        Tokens.theme = "no-such-theme"
        compare(Tokens.canvas.toString(), "#212121")
    }

    function test_every_theme_defines_every_colour() {
        var names = ["canvas", "surface", "surfaceGlass", "border", "separator", "text", "textTelemetry",
                     "textMuted", "accent", "accentText", "textOnAccent", "focus", "hover",
                     "stateWarning", "stateCaution", "stateAdvisory", "stateNormal", "stateGuidance",
                     "stateStandby", "stateInkWarning", "stateInkCaution", "stateInkAdvisory",
                     "stateInkNormal", "stateInkGuidance", "pfdSkyTop", "pfdSkyBottom", "pfdGroundTop",
                     "pfdGroundBottom", "pfdSymbol", "pfdHalo"]
        for (var i = 0; i < Tokens.themeKeys.length; ++i) {
            Tokens.theme = Tokens.themeKeys[i]
            for (var j = 0; j < names.length; ++j)
                verify(Tokens[names[j]].valid, Tokens.theme + "." + names[j])
        }
    }
}

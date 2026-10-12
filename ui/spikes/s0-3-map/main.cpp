// Spike S0-3 (T0-07): MapLibre Native drawn by Qt Quick through RHI (Metal on macOS).
//
// Flies a fixed camera route (continuous pan, zoom, rotation and tilt, set every frame) twice
// over a vector basemap and records the interval between presented frames. The first pass
// loads tiles ("cold"); the second reuses them ("warm"). Pass (ADR-0003): 60 fps full screen
// on the built-in Retina display; GPU load is read with `sudo powermetrics --samplers gpu_power` while it runs.
//
//   spike-s0-3-map [--seconds-per-pass 60] [--warmup 5] [--style URL] [--report PATH]
//                  [--fullscreen] [--manual]
//
// --manual skips the route: pan, pinch and rotate by hand (trackpad check); the report covers
// the whole session and is written when the window closes.

#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QFile>
#include <QGuiApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QSGRendererInterface>
#include <QScreen>
#include <QTimer>

#include <QMapLibre/Utils>

#include <algorithm>
#include <cstdio>
#include <vector>

namespace {

QString apiName(QSGRendererInterface::GraphicsApi api) {
    switch (api) {
        case QSGRendererInterface::Metal:
            return "Metal";
        case QSGRendererInterface::OpenGL:
            return "OpenGL";
        case QSGRendererInterface::Vulkan:
            return "Vulkan";
        case QSGRendererInterface::Direct3D11:
            return "Direct3D11";
        case QSGRendererInterface::Direct3D12:
            return "Direct3D12";
        case QSGRendererInterface::Software:
            return "Software";
        default:
            return "Other";
    }
}

/// Frame intervals of one measured phase, in milliseconds.
struct Phase {
    QString name;
    std::vector<double> intervals;
    double seconds = 0;

    QJsonObject report(double refreshHz) const {
        std::vector<double> v = intervals;
        std::sort(v.begin(), v.end());
        auto pct = [&](double q) {
            if (v.empty()) return 0.0;
            return v[static_cast<size_t>((v.size() - 1) * q + 0.5)];
        };
        const double period = 1000.0 / refreshHz;
        // A frame that took longer than 1.5 refresh periods means at least one missed vsync.
        const auto missed = std::count_if(v.begin(), v.end(), [&](double i) { return i > 1.5 * period; });
        const auto over25 = std::count_if(v.begin(), v.end(), [](double i) { return i > 25.0; });
        const double fps = seconds > 0 ? v.size() / seconds : 0;
        return QJsonObject{
            {"phase", name},
            {"seconds", seconds},
            {"frames", static_cast<qint64>(v.size())},
            {"fps", fps},
            {"interval_ms", QJsonObject{{"p50", pct(0.50)}, {"p95", pct(0.95)}, {"p99", pct(0.99)},
                                        {"max", v.empty() ? 0.0 : v.back()}}},
            {"missed_vsync_frames", static_cast<qint64>(missed)},
            {"frames_over_25ms", static_cast<qint64>(over25)},
            {"pass_60fps", fps >= 59.0 && missed <= static_cast<qint64>(v.size() / 100)},
        };
    }
};

} // namespace

int main(int argc, char *argv[]) {
    // Let QMapLibre pick the backend it was built for (Metal on macOS) and make Qt Quick use it.
    const QMapLibre::RendererType rendererType = QMapLibre::supportedRendererType();
    QQuickWindow::setGraphicsApi(static_cast<QSGRendererInterface::GraphicsApi>(rendererType));

    QGuiApplication app(argc, argv);
    QCommandLineParser cli;
    cli.addHelpOption();
    cli.addOptions({
        {"seconds-per-pass", "Seconds per pass of the route (default 60).", "s", "60"},
        {"warmup", "Seconds before the first pass (default 5).", "s", "5"},
        {"style", "Style URL.", "url", "https://tiles.openfreemap.org/styles/liberty"},
        {"report", "Write the JSON report here.", "path"},
        {"fullscreen", "Full screen (use for the measurement)."},
        {"manual", "No route: interact by hand; report on close."},
    });
    cli.process(app);
    const double passSeconds = cli.value("seconds-per-pass").toDouble();
    const int warmupMs = static_cast<int>(cli.value("warmup").toDouble() * 1000);
    const bool manual = cli.isSet("manual");

    QQmlApplicationEngine engine;
#ifdef S03_MAPLIBRE_QML
    engine.addImportPath(QStringLiteral(S03_MAPLIBRE_QML));
#endif
    engine.setInitialProperties({
        {"styleUrl", cli.value("style")},
        {"passSeconds", passSeconds},
        {"manual", manual},
        {"fullscreen", cli.isSet("fullscreen")},
    });
    engine.loadFromModule("S03", "Main");
    if (engine.rootObjects().isEmpty()) return 1;
    auto *window = qobject_cast<QQuickWindow *>(engine.rootObjects().first());
    if (window == nullptr) return 1;

    std::vector<Phase> phases;
    Phase *current = nullptr;
    QElapsedTimer sinceLast;
    QElapsedTimer phaseClock;
    QObject::connect(window, &QQuickWindow::frameSwapped, window, [&] {
        if (current != nullptr && sinceLast.isValid()) {
            current->intervals.push_back(sinceLast.nsecsElapsed() / 1e6);
        }
        sinceLast.restart();
    });
    auto startPhase = [&](const QString &name) {
        if (current != nullptr) current->seconds = phaseClock.nsecsElapsed() / 1e9;
        phases.push_back(Phase{name, {}, 0});
        current = &phases.back();
        phaseClock.restart();
        sinceLast.invalidate();
        // The route restarts with each phase.
        QMetaObject::invokeMethod(window, "startRoute");
    };

    auto finish = [&] {
        if (current != nullptr) current->seconds = phaseClock.nsecsElapsed() / 1e9;
        current = nullptr;
        QScreen *screen = window->screen();
        const double refresh = screen != nullptr && screen->refreshRate() > 0 ? screen->refreshRate() : 60.0;
        QJsonArray results;
        for (const auto &p : phases) results.append(p.report(refresh));
        const QJsonObject report{
            {"spike", "S0-3"},
            {"graphics_api", apiName(window->rendererInterface()->graphicsApi())},
            {"style", cli.value("style")},
            {"window_px", QJsonArray{window->width() * window->devicePixelRatio(),
                                     window->height() * window->devicePixelRatio()}},
            {"device_pixel_ratio", window->devicePixelRatio()},
            {"screen", screen != nullptr ? screen->name() : QString()},
            {"screen_px", screen != nullptr ? QJsonArray{screen->size().width() * screen->devicePixelRatio(),
                                                         screen->size().height() * screen->devicePixelRatio()}
                                            : QJsonArray{}},
            {"refresh_hz", refresh},
            {"qt", qVersion()},
            {"manual", manual},
            {"phases", results},
            {"gpu_percent", "read separately: sudo powermetrics --samplers gpu_power -i 1000"},
        };
        const QByteArray json = QJsonDocument(report).toJson(QJsonDocument::Compact);
        std::printf("%s\n", json.constData());
        if (cli.isSet("report")) {
            QFile f(cli.value("report"));
            if (f.open(QIODevice::WriteOnly)) f.write(json + "\n");
        }
    };

    if (manual) {
        QTimer::singleShot(warmupMs, window, [&] { startPhase("manual"); });
        QObject::connect(&app, &QGuiApplication::aboutToQuit, finish);
    } else {
        const int passMs = static_cast<int>(passSeconds * 1000);
        QTimer::singleShot(warmupMs, window, [&] { startPhase("cold"); });
        QTimer::singleShot(warmupMs + passMs, window, [&] { startPhase("warm"); });
        QTimer::singleShot(warmupMs + 2 * passMs, window, [&] {
            finish();
            QGuiApplication::quit();
        });
    }
    return QGuiApplication::exec();
}

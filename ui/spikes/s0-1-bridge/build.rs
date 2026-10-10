use cxx_qt_build::{CxxQtBuilder, QmlModule};

fn main() {
    CxxQtBuilder::new_qml_module(
        QmlModule::new("org.drorcontrol.spike").qml_files(["qml/main.qml"]),
    )
    .qt_module("Quick")
    .files(["src/telemetry.rs"])
    .build();
}

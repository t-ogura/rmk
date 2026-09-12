//! Utilities of check cargo feature
//!

/// Get enabled RMK features list.
///
/// The list is read from the `rmk` entry in the user crate's `Cargo.toml`.
/// A crate that forwards rmk features through its own `[features]` (for
/// example `vial = ["rmk/vial"]`, selected with `--features`) is invisible
/// there, so its build script can name the forwarded features in the
/// `RMK_FEATURES` environment variable (comma separated, via
/// `cargo:rustc-env=RMK_FEATURES=vial,host_lock`); those are added.
pub(crate) fn get_rmk_features() -> Option<Vec<String>> {
    // Use an absolute path. `cargo_toml::Manifest::from_path` resolves the
    // workspace root by walking ancestors of the given path; passing a
    // relative `"./Cargo.toml"` makes its fallback canonicalize an empty
    // path and fail with ENOENT inside workspace members.
    let manifest_path = std::path::Path::new(
        &std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set"),
    )
    .join("Cargo.toml");
    match cargo_toml::Manifest::from_path(&manifest_path) {
        Ok(manifest) => manifest
            .dependencies
            .iter()
            .find(|(name, _dep)| *name == "rmk")
            .map(|(_name, dep)| {
                let default_features = if let Some(d) = dep.detail() {
                    d.default_features
                } else {
                    true
                };

                let mut feature_set = dep.req_features().to_vec();

                // Add default features to the feature list.
                // Must mirror `default = [...]` in rmk/Cargo.toml.
                if default_features {
                    feature_set.push("defmt".to_string());
                    feature_set.push("storage".to_string());
                    feature_set.push("vial".to_string());
                    feature_set.push("host_lock".to_string());
                    feature_set.push("watchdog".to_string());
                }
                if let Ok(forwarded) = std::env::var("RMK_FEATURES") {
                    feature_set.extend(
                        forwarded
                            .split(',')
                            .map(str::trim)
                            .filter(|f| !f.is_empty())
                            .map(String::from),
                    );
                }
                feature_set
            }),
        Err(_e) => None,
    }
}

/// Check whether the given feature is enabled
pub(crate) fn is_feature_enabled(feature_list: &Option<Vec<String>>, feature: &str) -> bool {
    if let Some(rmk_features) = feature_list {
        for f in rmk_features {
            if f == feature {
                return true;
            }
        }
    }
    false
}

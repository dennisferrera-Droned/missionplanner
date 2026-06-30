-keep class com.dji.photogrammetry.app.** { *; }
-keepattributes *Annotation*
-keepattributes SourceFile,LineNumberTable

# ── DJI MSDK v5 ─────────────────────────────────────────────────────────────
-keep class dji.** { *; }
-keep class com.dji.** { *; }
-dontwarn dji.**
-dontwarn com.dji.**
-keep class net.sqlcipher.** { *; }
-keep interface dji.** { *; }
-keep interface com.dji.** { *; }

# ── OsmDroid ─────────────────────────────────────────────────────────────────
-keep class org.osmdroid.** { *; }
-dontwarn org.osmdroid.**

# ── Gson (used by MissionConverter) ─────────────────────────────────────────
-keep class com.google.gson.** { *; }
-keepclassmembers class * {
    @com.google.gson.annotations.SerializedName <fields>;
}

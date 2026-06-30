package com.dji.photogrammetry.app

import android.app.Application

/**
 * Application entry point.
 *
 * DJI MSDK v5 must be initialised here (not in an Activity) so the SDK
 * registers before any UI component tries to use it.
 */
class App : Application() {

    override fun onCreate() {
        super.onCreate()
        DjiSdkManager.init(this)
    }
}

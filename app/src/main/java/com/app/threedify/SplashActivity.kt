package com.app.threedify

import android.content.Context
import android.content.Intent
import android.graphics.drawable.AnimationDrawable
import android.os.Bundle
import android.os.Handler
import android.widget.ImageView
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.app.AppCompatDelegate

class SplashActivity : AppCompatActivity() {

    private lateinit var splashAnimation: AnimationDrawable

    override fun onCreate(savedInstanceState: Bundle?) {
        applySavedTheme()
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_splash)

        val splashImage = findViewById<ImageView>(R.id.splashImage)
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        val isDarkTheme = sharedPreferences.getBoolean("isDarkTheme", false)

        if (isDarkTheme) {
            splashImage.setBackgroundResource(R.drawable.animation)
        } else {
            splashImage.setBackgroundResource(R.drawable.animation_white_them)
        }

        splashAnimation = splashImage.background as AnimationDrawable

        splashImage.post { splashAnimation.start() }

        Handler().postDelayed({
            startActivity(Intent(this, MainActivity::class.java))
            finish()
        }, 2000)
    }

    private fun applySavedTheme() {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        val isDarkTheme = sharedPreferences.getBoolean("isDarkTheme", false)
        if (isDarkTheme) {
            AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES)
        } else {
            AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO)
        }
    }
}
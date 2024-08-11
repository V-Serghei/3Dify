package com.app.threedify

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.graphics.drawable.AnimationDrawable
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.widget.ImageView
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.app.AppCompatDelegate

@SuppressLint("CustomSplashScreen")
class SplashActivity : AppCompatActivity() {

    ///////////////////////////////////////////////////////////////////
    //Constant animation duration
    companion object {
        private const val SPLASH_DURATION = 2000L
    }

    private lateinit var splashAnimation: AnimationDrawable

    override fun onCreate(savedInstanceState: Bundle?) {
        applySavedTheme()
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_splash)

        val splashImage = findViewById<ImageView>(R.id.splashImage)
        splashImage?.let {
            setupSplashAnimation(it)
        }

        // Using a constant for delay for full animation at least once
        Handler(Looper.getMainLooper()).postDelayed({
            navigateToMainActivity()
        }, SPLASH_DURATION)
    }


    ///////////////////////////////////////////////////////////////////
    //Applies the saved theme to the activity based on user preference.
    //It checks if the dark theme was previously selected and sets the
    //appropriate night mode.
    private fun applySavedTheme() {
        val isDarkTheme = getSavedThemeState()
        AppCompatDelegate.setDefaultNightMode(
            if (isDarkTheme) AppCompatDelegate.MODE_NIGHT_YES else AppCompatDelegate.MODE_NIGHT_NO
        )
    }

    ///////////////////////////////////////////////////////////////////
    //Retrieves the saved theme state from SharedPreferences.
    private fun getSavedThemeState(): Boolean {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        return sharedPreferences.getBoolean("isDarkTheme", false)
    }

    ///////////////////////////////////////////////////////////////////
    //Sets up the splash screen animation based on the current theme.
    private fun setupSplashAnimation(imageView: ImageView) {
        val animationResource = if (getSavedThemeState()) {
            R.drawable.animation
        } else {
            R.drawable.animation_white_them
        }
        imageView.setBackgroundResource(animationResource)
        splashAnimation = imageView.background as? AnimationDrawable ?: return
        imageView.post { splashAnimation.start() }
    }

    ///////////////////////////////////////////////////////////////////
    //Starts the MainActivity
    private fun navigateToMainActivity() {
        startActivity(Intent(this, MainActivity::class.java))
        finish()
    }


}

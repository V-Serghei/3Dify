package com.app.threedify

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Menu
import android.view.ViewGroup
import android.widget.Switch
import androidx.activity.OnBackPressedCallback
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.app.AppCompatDelegate
import androidx.core.view.GravityCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.core.view.updateLayoutParams
import androidx.drawerlayout.widget.DrawerLayout
import androidx.navigation.NavController
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.NavOptions
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.navigateUp
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.app.threedify.databinding.ActivityMainBinding
import com.google.android.material.navigation.NavigationView

import org.the3deer.app.model3D.view.ModelActivity
import org.the3deer.util.android.AndroidURLStreamHandlerFactory
import java.net.URL

class MainActivity : AppCompatActivity() {

    /////////////////////////////////////////////////////////////////
    //Should exist in every class which uses activities from VIEWERAPP
    companion object {
        init {
            System.setProperty("java.protocol.handler.pkgs", "org.the3deer.util.android")
            URL.setURLStreamHandlerFactory(AndroidURLStreamHandlerFactory())
        }
    }
    /////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    ///for interface!!
    private lateinit var appBarConfiguration: AppBarConfiguration
    private lateinit var binding: ActivityMainBinding
    @SuppressLint("UseSwitchCompatOrMaterialCode")
    private lateinit var switchTheme: Switch
    private lateinit var navController: NavController
    ///for interface!!
    ////////////////////////////////////////////////////////////////

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        ////////////////////////////////////////////////////////////////
        ///for interface!!
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.appBarMain.toolbar)
        navController = findNavController(R.id.nav_host_fragment_content_main)


        navController = findNavController(R.id.nav_host_fragment_content_main)
        val drawerLayout: DrawerLayout = binding.main
        val navView: NavigationView = binding.navView
        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home, R.id.nav_camera, R.id.nav_gallery, R.id.nav_settings, R.id.nav_about_us,R.id.nav_home
            ), drawerLayout
        )
        binding.appBarMain.fab.setOnClickListener {
            openCameraFragment()

            Handler(Looper.getMainLooper()).postDelayed({
                recreateStackAppBarConfiguration(drawerLayout, navView)
            }, 100)
        }

        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)

        switchTheme = navView.findViewById(R.id.switch_theme)
        switchTheme.isChecked = getSavedThemeState()
        switchTheme.setOnCheckedChangeListener { _, isChecked ->
            if (isChecked) {
                setDarkTheme()
            } else {
                setLightTheme()
            }
            saveThemeState(isChecked)
        }
        if (switchTheme.isChecked) {
            setDarkTheme()
        } else {
            setLightTheme()
        }
        //so that on action "back" hides menu
        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                if (drawerLayout.isDrawerOpen(GravityCompat.START)) {
                    drawerLayout.closeDrawer(GravityCompat.START)
                } else {
                    onBackPressedDispatcher.onBackPressed()
                }
            }
        })
        ///for interface!!
        ////////////////////////////////////////////////////////////////

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }
        //so that app runs under status bar
        ViewCompat.setOnApplyWindowInsetsListener(drawerLayout) { view, insets ->
            val systemBarInsets = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            view.updateLayoutParams<ViewGroup.MarginLayoutParams> {
                topMargin = systemBarInsets.top
                bottomMargin = systemBarInsets.bottom
            }
            WindowInsetsCompat.CONSUMED
        }
        //!!if u want to hide the status bar just remove comment from here
/*                WindowInsetsControllerCompat(window, drawerLayout).let { controller ->
                    controller.hide(WindowInsetsCompat.Type.statusBars())
                    controller.systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
                }*/


        /////////////////////////////////////////////////////////////////
        //Call for 3D viewer window
        //prefix - "android://"
        //package - "org.andresoviedo.dddmodel2/assets/models/"
        //file - cube.obj
//        val intent = Intent(this, ModelActivity::class.java)
//        intent.putExtra("uri", "android://org.andresoviedo.dddmodel2/assets/models/cube.obj")
//        intent.putExtra("type", "-1")
//        intent.putExtra("immersiveMode", "false")
//        intent.putExtra("backgroundColor", "1.0 1.0 1.0 1.0")
//        startActivity(intent)
        /////////////////////////////////////////////////////////////////
    }

    ////////////////////////////////////////////////////////////////
    ///for interface!!
    private fun recreateStackAppBarConfiguration(drawerLayout: DrawerLayout, navView: NavigationView) {
        navController = findNavController(R.id.nav_host_fragment_content_main)
        navController.clearBackStack(R.id.nav_home)
        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home, R.id.nav_camera, R.id.nav_gallery, R.id.nav_settings, R.id.nav_about_us
            ), drawerLayout
        )
        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)
    }



    ////////////////////////////////////////////////////////////////
    ///for interface!!
    private fun openCameraFragment() {
        navController = findNavController(R.id.nav_host_fragment_content_main)
        val navOptions = NavOptions.Builder()
            .setPopUpTo(navController.graph.findStartDestination().id, inclusive = true) // Clear back stack up to start destination
            .build()

        navController.navigate(R.id.nav_camera, null, navOptions)
    }



    private fun setLightTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO)
    }

    private fun setDarkTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES)
    }

    private fun getSavedThemeState(): Boolean {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        return sharedPreferences.getBoolean("isDarkTheme", false)
    }

    private fun saveThemeState(isDarkTheme: Boolean) {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putBoolean("isDarkTheme", isDarkTheme)
        editor.apply()
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menuInflater.inflate(R.menu.main, menu)

        return true
    }

    override fun onSupportNavigateUp(): Boolean {
        return navController.navigateUp(appBarConfiguration) || super.onSupportNavigateUp()
    }

    ///for interface!!
    ////////////////////////////////////////////////////////////////
}
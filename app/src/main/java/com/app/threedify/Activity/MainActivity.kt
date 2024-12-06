package com.app.threedify.Activity

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.os.Bundle
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
import androidx.core.view.updateLayoutParams
import androidx.drawerlayout.widget.DrawerLayout
import androidx.navigation.NavController
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.NavigationUI
import androidx.navigation.ui.navigateUp
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.app.arcore.ArCoreManager
import com.app.threedify.helpers.NavigationState
import com.app.threedify.R
import com.app.threedify.databinding.ActivityMainBinding
import com.chaquo.python.Python
import com.chaquo.python.android.AndroidPlatform
import com.google.android.material.navigation.NavigationView
import org.the3deer.app.model3D.view.ModelActivity
import org.the3deer.util.android.AndroidURLStreamHandlerFactory
import java.net.URL

class MainActivity : AppCompatActivity() {

    /////////////////////////////////////////////////////////////////
    //Should exist in every class which uses activities from VIEWERAPP
    //!!!StreamHandlerFactory can be declared only once per program!!!
    companion object {
        init {
            System.setProperty("java.protocol.handler.pkgs", "org.the3deer.util.android")
            URL.setURLStreamHandlerFactory(AndroidURLStreamHandlerFactory())
        }
    }
    /////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    ///for interface!!
    private lateinit var appBarConfiguration: AppBarConfiguration
    private lateinit var binding: ActivityMainBinding
    @SuppressLint("UseSwitchCompatOrMaterialCode")
    private lateinit var switchTheme: Switch
    private lateinit var navController: NavController
    private var savedNavigationState: NavigationState = NavigationState.HOME
    ///for interface!!
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////
        ///for interface!!

        if (!Python.isStarted()) {
            Python.start(AndroidPlatform(this))
        }

        // initialize views
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.appBarMain.toolbar)

        // Initialize navigation controller and setup
        navController = findNavController(R.id.nav_host_fragment_content_main)
        setupDrawer()
        // Initialize theme switch
        setupThemeSwitch()
        // Handle back press actions
        handleBackPress()
        // Setup window insets to handle system bars
        setupWindowInsets()

        ///for interface!!
        ////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////

        // Optional: Uncomment to initialize 3D viewer
        // initialize3DViewer()

    }


    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////
    ///for interface!!

    private fun setupDrawer() {
        //Setup drawer layout and navigation view
        val drawerLayout: DrawerLayout = binding.main
        val navView: NavigationView = binding.navView

        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home,
                R.id.nav_camera,
                R.id.nav_gallery,
                R.id.nav_settings,
                R.id.nav_about_us
            ),
            drawerLayout
        )

        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)

        navView.setNavigationItemSelectedListener { menuItem ->
            when (menuItem.itemId) {
                R.id.nav_camera -> {
                    openCameraActivity(fromMenu = true)
                    true
                }
                else -> {
                    val handled = NavigationUI.onNavDestinationSelected(menuItem, navController)
                    if (handled) {
                        drawerLayout.closeDrawer(GravityCompat.START)
                    }
                    handled
                }
            }
        }
        binding.appBarMain.fab.setOnClickListener {
            openCameraActivity(fromMenu = false)
        }

    }


    private fun setupThemeSwitch() {
        // Initialize theme switch and set its state based on saved preference
        switchTheme = binding.navView.findViewById(R.id.switch_theme)
        switchTheme.isChecked = getSavedThemeState()
        switchTheme.setOnCheckedChangeListener { _, isChecked ->
            // Change theme based on switch state
            if (isChecked) setDarkTheme() else setLightTheme()
            saveThemeState(isChecked)
        }
    }

    // Handle back press to close the drawer if open, otherwise perform default back press action
    private fun handleBackPress() {
        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                val drawerLayout: DrawerLayout = binding.main
                if (drawerLayout.isDrawerOpen(GravityCompat.START)) {
                    drawerLayout.closeDrawer(GravityCompat.START)
                } else {
                    onBackPressedDispatcher.onBackPressed()
                }
            }
        })
    }

    // Adjust padding for main view based on system bars
    private fun setupWindowInsets() {
        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main)) { v, insets ->
            val systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom)
            insets
        }
        // Adjust margins for drawer layout based on system bars
        ViewCompat.setOnApplyWindowInsetsListener(binding.main) { view, insets ->
            val systemBarInsets = insets.getInsets(WindowInsetsCompat.Type.systemBars())
            view.updateLayoutParams<ViewGroup.MarginLayoutParams> {
                topMargin = systemBarInsets.top
                bottomMargin = systemBarInsets.bottom
            }
            WindowInsetsCompat.CONSUMED
        }
    }

    // Clear back stack and recreate app bar configuration
    private fun recreateStackAppBarConfiguration(drawerLayout: DrawerLayout, navView: NavigationView) {
        navController = findNavController(R.id.nav_host_fragment_content_main)
        navController.clearBackStack(R.id.nav_home)
        appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.nav_home,
                R.id.nav_camera,
                R.id.nav_gallery,
                R.id.nav_settings,
                R.id.nav_about_us
            ),
            drawerLayout
        )
        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)
    }


    private fun openCameraActivity(fromMenu: Boolean) {
        saveCurrentNavigationState()
        val intent = Intent(this, RawDepthCodelabActivity::class.java)
        intent.putExtra("fromMenu", fromMenu)
        startActivity(intent)
    }

    // Set app theme to light mode
    private fun setLightTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_NO)
    }

    // Set app theme to dark mode
    private fun setDarkTheme() {
        AppCompatDelegate.setDefaultNightMode(AppCompatDelegate.MODE_NIGHT_YES)
    }

    // Retrieve saved theme state from shared preferences
    private fun getSavedThemeState(): Boolean {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        return sharedPreferences.getBoolean("isDarkTheme", false)
    }

    // Save theme state to shared preferences
    private fun saveThemeState(isDarkTheme: Boolean) {
        val sharedPreferences = getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putBoolean("isDarkTheme", isDarkTheme)
        editor.apply()
    }

    // Set the options menu
    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menuInflater.inflate(R.menu.main, menu)
        return true
    }

    // Handle navigation up action
    override fun onSupportNavigateUp(): Boolean {
        return navController.navigateUp(appBarConfiguration) || super.onSupportNavigateUp()
    }
    ///for interface!!
    ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////


//        override fun onRequestPermissionsResult(
//        requestCode: Int, permissions: Array<String>, grantResults: IntArray
//    ) {
//        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
//        val arCoreManager = ArCoreManager()
//        arCoreManager.initialize(this, findViewById(R.id.ar_scene_view), this)
//        arCoreManager.onRequestPermissionsResult(requestCode, permissions, grantResults)
//    }

    override fun onResume() {
        super.onResume()
        restoreNavigationState()
        val navView: NavigationView = binding.navView
        val menu = navView.menu
        val currentFragmentId = when (savedNavigationState) {
            NavigationState.HOME -> R.id.nav_home
            NavigationState.CAMERA -> R.id.nav_camera
            NavigationState.GALLERY -> R.id.nav_gallery
            NavigationState.SETTINGS -> R.id.nav_settings
            NavigationState.ABOUT_US -> R.id.nav_about_us
        }
        for (i in 0 until menu.size()) {
            menu.getItem(i).isChecked = menu.getItem(i).itemId == currentFragmentId
        }
    }
    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        intent.removeExtra("fromMenu")
    }
    private fun saveCurrentNavigationState() {
        savedNavigationState = when (navController.currentDestination?.id) {
            R.id.nav_home -> NavigationState.HOME
            R.id.nav_camera -> NavigationState.CAMERA
            R.id.nav_gallery -> NavigationState.GALLERY
            R.id.nav_settings -> NavigationState.SETTINGS
            R.id.nav_about_us -> NavigationState.ABOUT_US
            else -> NavigationState.HOME
        }
    }

    private fun restoreNavigationState() {
        when (savedNavigationState) {
            NavigationState.HOME -> navController.navigate(R.id.nav_home)
            NavigationState.CAMERA -> navController.navigate(R.id.nav_camera)
            NavigationState.GALLERY -> navController.navigate(R.id.nav_gallery)
            NavigationState.SETTINGS -> navController.navigate(R.id.nav_settings)
            NavigationState.ABOUT_US -> navController.navigate(R.id.nav_about_us)
        }
    }


}

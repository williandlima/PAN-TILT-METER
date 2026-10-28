plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose")
    id("com.google.devtools.ksp")
}

android {
    namespace = "com.williandlima.inclinometro"
    compileSdk = 34

    defaultConfig {
        // O applicationId e' a IDENTIDADE do app para o Android: dois APKs com
        // o mesmo id nao coexistem — instalar um substitui o outro. Este e'
        // diferente do da versao 1 (so inclinacao, `com.williandlima.
        // inclinometro`) de proposito, para as duas poderem ficar instaladas
        // ao mesmo tempo no aparelho, cada uma com seus proprios dados
        // (o banco Room fica no sandbox privado de cada applicationId).
        //
        // O `namespace` acima continua o mesmo — ele so define o pacote das
        // classes geradas (R, BuildConfig) e nao precisa acompanhar. A
        // authority do FileProvider usa `${applicationId}` no manifesto, entao
        // se ajusta sozinha.
        applicationId = "com.williandlima.inclinometro.doiseixos"
        minSdk = 26
        targetSdk = 34
        versionCode = 4
        versionName = "2.2.0"
    }

    buildFeatures {
        compose = true
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    kotlinOptions {
        jvmTarget = "17"
    }
}

dependencies {
    implementation(platform("androidx.compose:compose-bom:2024.09.02"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.ui:ui-tooling-preview")
    implementation("androidx.activity:activity-compose:1.9.2")
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.6")
    implementation("androidx.lifecycle:lifecycle-viewmodel-compose:2.8.6")

    implementation("androidx.room:room-runtime:2.6.1")
    implementation("androidx.room:room-ktx:2.6.1")
    ksp("androidx.room:room-compiler:2.6.1")

    debugImplementation("androidx.compose.ui:ui-tooling")
}

#include <iostream>
#include "rom.h"
#include "cpu.h"
#include "engine.h"

#include <chrono>
#include <glad/glad.h>

#if defined(_WIN32) || defined(__APPLE__)
    #include <SDL.h>
    #include <SDL_opengl.h>
#elif defined(__unix__)
    #include <SDL2/SDL.h>
    #include <SDL2/SDL_opengl.h>
#endif

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"

#define WINDEBUG 0

int main(int argc, char** argv) {
    // Force cout/stdio to use a buffer.
    // Used for msvc because it doesn't buffer automatically. Without this, enabling cpu printing
    // causes msvc to take 10 seconds to run through nestest.nes. With this, it takes 2.5 seconds.
    // g++ takes 50 milliseconds.
    setvbuf(stdout, 0, _IOLBF, 4096);
    std::cout.sync_with_stdio(false);

    Engine engine(false);
    engine.load("tests/nestest.nes", 0xC000);
    //auto start = std::chrono::high_resolution_clock::now();
#ifdef __unix__
    while (!engine.endRunning())
    {
        engine.run();
    }
#elif WINDEBUG == 1
    while (!engine.endRunning())
    {
        engine.run();
    }
#elif defined(_WIN32) || defined(__APPLE__)

    const char* glsl_version = "#version 330";
    SDL_Window* window = NULL;
    SDL_Surface* surface = NULL;
    SDL_Event event;
    bool quit = false;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) 
    {
        std::cout << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
    }
    else
    {
        // Set opengl version, core profile.
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

        // Create window with graphics context
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
        SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        SDL_Window* window = SDL_CreateWindow("mageNES", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, window_flags);
        SDL_GLContext gl_context = SDL_GL_CreateContext(window);
        if (SDL_GL_MakeCurrent(window, gl_context) != 0) {
            std::cout << "Failed to make current context for window. SDL_Error: " << SDL_GetError() << std::endl;
        };
        //SDL_GL_SetSwapInterval(1); // Enable vsync

        if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress))
        {
            std::cout << "Failed to initialize GLAD." << std::endl;
            return -1;
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();
        //ImGui::StyleColorsClassic();

        // Setup Platform/Renderer bindings
        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init(glsl_version);

        // Our state
        bool show_demo_window = true;
        bool show_another_window = false;
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

        while (!quit)
        {
            // Need to run the engine for 1 frame instead of 1 cycle.
            engine.run();
            while (SDL_PollEvent(&event))
            {
                ImGui_ImplSDL2_ProcessEvent(&event);
                if (event.type == SDL_KEYDOWN)
                {
                    switch (event.key.keysym.sym)
                    {
                    case SDLK_ESCAPE:
                        quit = true;
                        break;
                    default:
                        break;
                    }
                } else if (event.type == SDL_QUIT)
                {
                    quit = true;
                } else if (event.type == SDL_WINDOWEVENT)
				{
					if (event.window.event == SDL_WINDOWEVENT_RESIZED) 
					{
                        // Resize
						//viewportWidth = event.window.data1;
						//viewportHeight = event.window.data2;
					}
				}
            }

            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame(window);
            ImGui::NewFrame();

            // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
            if (show_demo_window)
                ImGui::ShowDemoWindow(&show_demo_window);

            // Rendering
            ImGui::Render();
            glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
            glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
            SDL_GL_SwapWindow(window);
        }

        // Cleanup
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();

        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

#endif
    //auto stop = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    /*std::cout << "Time taken by function: "
        << duration.count() << " milliseconds" << std::endl;*/
    std::cout << "Finished." << std::endl;
    std::cin.get();
    return 0;
}
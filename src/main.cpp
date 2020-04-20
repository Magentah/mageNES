#include <iostream>
#include "rom.h"
#include "cpu.h"
#include "engine.h"

#include <glad/glad.h>
#ifdef _WIN32
    #include <SDL.h>
    #include <SDL_opengl.h>
#elif __unix__
    #include <SDL2/SDL.h>
    #include <SDL2/SDL_opengl.h>
#endif

#define WINDEBUG 1

int main(int argc, char** argv) {
    Engine engine;
    engine.load("tests/nestest.nes", 0xC000);

#ifdef __unix__
    while(engine.run())
    {
        //engine.run();
    }
#elif WINDEBUG == 1
    while (engine.run())
    {
        //engine.run();
    }
#elif _WIN32

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
        auto windowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_SHOWN;
        window = SDL_CreateWindow("mageNES", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 600, 400, windowFlags);
        SDL_GLContext context = SDL_GL_CreateContext(window);
        if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress))
        {
            std::cout << "Failed to initialize GLAD." << std::endl;
            return -1;
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        while (!quit)
        {
            engine.run();
            while (SDL_PollEvent(&event))
            {
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
        }
    }
    
    SDL_DestroyWindow(window);

	//Quit SDL subsystems
	SDL_Quit();

#endif

    return 0;
}
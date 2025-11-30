/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 *
 */

#include <interrupts_101308485_101297581.hpp>

void FCFS(std::vector<PCB> &ready_queue)
{
    std::sort(
        ready_queue.begin(),
        ready_queue.end(),
        [](const PCB &first, const PCB &second)
        {
            return (first.arrival_time > second.arrival_time);
        });
}

int find_highest_priority_index(const std::vector<PCB> &ready_queue)
{
    int best_idx = 0;
    int best_prio = ready_queue[0].priority;

    for (int i = 1; i < (int)ready_queue.size(); ++i)
    {
        if (ready_queue[i].priority < best_prio)
        {
            best_prio = ready_queue[i].priority;
            best_idx = i;
        }
    }
    return best_idx;
}

std::tuple<std::string, std::string> run_simulation(std::vector<PCB> list_processes)
{

    std::vector<PCB> ready_queue; // The ready queue of processes
    std::vector<PCB> wait_queue;  // The wait queue of processes
    std::vector<PCB> job_list;    // A list to keep track of all the processes. This is similar
                                  // to the "Process, Arrival time, Burst time" table that you
                                  // see in questions. You don't need to use it, I put it here
                                  // to make the code easier :).

    unsigned int current_time = 0;
    PCB running;

    // Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;
    std::string memory_log;

    // make the output table (the header row)
    execution_status = print_exec_header();

    unsigned int quantum_used = 0;

    // Loop while till there are no ready or waiting processes.
    // This is the main reason I have job_list, you don't have to use it.
    while (!all_process_terminated(job_list) || job_list.empty())
    {

        // Inside this loop, there are three things you must do:
        //  1) Populate the ready queue with processes as they arrive
        //  2) Manage the wait queue
        //  3) Schedule processes from the ready queue

        // Population of ready queue is given to you as an example.
        // Go through the list of proceeses
        for (auto &process : list_processes)
        {
            if (process.arrival_time == current_time)
            { // check if the AT = current time
                // if so, assign memory and put the process into the ready queue
                bool allocated = assign_memory(process);
                if (allocated)
                {
                    // process can be admitted to memory then NEW to READY
                    process.state = READY;
                    ready_queue.push_back(process);
                    job_list.push_back(process);
                    execution_status += print_exec_status(current_time, process.PID, NEW, READY);
                    memory_log += memory_status();
                }
                else
                {
                    // process has arrived but no memory yet so stay NEW, add to job_list
                    process.state = NEW;
                    job_list.push_back(process);
                    memory_log += "Process " + std::to_string(process.PID) + " could not be admitted at time " + std::to_string(current_time) + " (no memory).\n\n";
                }
            }
        }
        // Update I/O (WAITING to READY when done)
        for (auto it = wait_queue.begin(); it != wait_queue.end();)
        {

            if (it->io_remaining > 0)
            {
                it->io_remaining--;
            }

            if (it->io_remaining == 0)
            {
                // I/O complete: WAITING to READY
                execution_status += print_exec_status(current_time, it->PID, WAITING, READY);
                it->state = READY;
                it->time_since_last_io = 0; // reset for next I/O

                ready_queue.push_back(*it);
                sync_queue(job_list, *it);
                it = wait_queue.erase(it);
            }
            else
            {
                ++it;
            }
        }
        for (auto &job : job_list)
        {
            if (job.state == NEW && job.partition_number == -1)
            {
                bool allocated = assign_memory(job);

                if (allocated)
                {
                    job.state = READY;
                    ready_queue.push_back(job);

                    execution_status += print_exec_status(current_time, job.PID, NEW, READY);
                    memory_log += memory_status();
                }
            }
        }
        //  If CPU is idle, dispatch next ready process
        if (running.PID == -1 && !ready_queue.empty())
        {
            int idx = find_highest_priority_index(ready_queue);
            running = ready_queue[idx];
            ready_queue.erase(ready_queue.begin() + idx);

            states old_state = running.state;
            running.state = RUNNING;
            if (running.start_time == -1)
            {
                running.start_time = current_time;
            }
            quantum_used = 0;

            execution_status += print_exec_status(current_time, running.PID, old_state, RUNNING);
            sync_queue(job_list, running);
        }
        if (running.PID != -1 && running.state == RUNNING && !ready_queue.empty())
        {
            int idx = find_highest_priority_index(ready_queue);

            // smaller priority value = higher priority
            if (ready_queue[idx].priority < running.priority)
            {
                // current running process goes back to READY
                execution_status += print_exec_status(current_time, running.PID, RUNNING, READY);
                running.state = READY;
                ready_queue.push_back(running);
                sync_queue(job_list, running);

                // dispatch the higher-priority one
                running = ready_queue[idx];
                ready_queue.erase(ready_queue.begin() + idx);

                states old_state = running.state;
                running.state = RUNNING;
                if (running.start_time == -1)
                {
                    running.start_time = current_time;
                }
                quantum_used = 0; // new gets 100ms

                execution_status += print_exec_status(current_time, running.PID, old_state, RUNNING);
                sync_queue(job_list, running);
            }
        }
        // Run 1ms of CPU time if we have a running process
        if (running.state == RUNNING)
        {

            running.remaining_time--;
            running.time_since_last_io++;
            quantum_used++;
            current_time++; // 1 ms passes

            // Process finished
            if (running.remaining_time == 0)
            {

                execution_status += print_exec_status(current_time, running.PID, RUNNING, TERMINATED);
                terminate_process(running, job_list); // frees memory
                memory_log += memory_status();
                idle_CPU(running); // CPU now idle
            }
            // I/O request
            else if (running.io_freq > 0 && running.time_since_last_io >= running.io_freq)
            {
                execution_status += print_exec_status(current_time, running.PID, RUNNING, WAITING);
                running.state = WAITING;
                running.io_remaining = running.io_duration + 1;
                running.time_since_last_io = 0;

                wait_queue.push_back(running);
                sync_queue(job_list, running);
                idle_CPU(running);
            }
            // Quantum expired
            else if (quantum_used >= 100)
            {

                execution_status += print_exec_status(current_time, running.PID, RUNNING, READY);
                running.state = READY;
                ready_queue.push_back(running);
                sync_queue(job_list, running);
                idle_CPU(running);
            }
        }
        else
        {
            // No process on CPU
            current_time++;
        }
    }
    execution_status += print_exec_footer();
    return std::make_tuple(execution_status, memory_log);
}

int main(int argc, char **argv)
{

    // Get the input file from the user
    if (argc != 2)
    {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    // Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    // Ensure that the file actually opens
    if (!input_file.is_open())
    {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    // Parse the entire input file and populate a vector of PCBs.
    // To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while (std::getline(input_file, line))
    {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    // With the list of processes, run the simulation
    auto [exec, memlog] = run_simulation(list_process);

    write_output(exec, "output_files/execution.txt");
    write_output(memlog, "output_files/memory.txt");

    return 0;
}